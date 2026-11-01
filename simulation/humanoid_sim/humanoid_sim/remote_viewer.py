"""Read-only network viewer. Client: python -m humanoid_sim.remote_viewer.

No ROS imports. MJB assets are sent once, followed by latest-only integration
states. Use an SSH tunnel or trusted LAN; this protocol has no authentication.
"""
from __future__ import annotations

import argparse
import json
import multiprocessing
from queue import Empty, Full
import socket
import struct
import threading
import time

import mujoco
import numpy as np

SPEC = mujoco.mjtState.mjSTATE_INTEGRATION
MAX_PACKET = 256 * 1024 * 1024


def _put_latest(queue, value):
    try:
        queue.put_nowait(value)
    except Full:
        try:
            queue.get_nowait()
        except Empty:
            pass
        try:
            queue.put_nowait(value)
        except Full:
            pass


def _send(sock, payload):
    sock.sendall(struct.pack('!I', len(payload)) + payload)


def _receive(sock, limit=MAX_PACKET):
    def exact(size):
        chunks = bytearray()
        while len(chunks) < size:
            part = sock.recv(min(size - len(chunks), 65536))
            if not part:
                raise ConnectionError('Remote viewer connection closed')
            chunks.extend(part)
        return bytes(chunks)
    size, = struct.unpack('!I', exact(4))
    if size > limit:
        raise ValueError(f'Remote packet exceeds limit: {size}')
    return exact(size)


def _serve(model, snapshots, stop, address, ready):
    """Only this process performs network IO and model serialization."""
    try:
        binary = np.empty(mujoco.mj_sizeModel(model), dtype=np.uint8)
        mujoco.mj_saveModel(model, buffer=binary)
        hello = json.dumps(dict(protocol=1, mujoco=mujoco.__version__,
                                state_size=mujoco.mj_stateSize(model, SPEC))).encode()
        with socket.socket() as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(address)
            listener.listen(1)
            listener.settimeout(0.2)
            ready.send((True, listener.getsockname()[1]))
            latest = None
            while not stop.is_set():
                try:
                    connection, _ = listener.accept()
                except socket.timeout:
                    continue
                with connection:
                    connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    connection.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 65536)
                    connection.settimeout(5.0)
                    try:
                        _send(connection, hello)
                        _send(connection, binary.tobytes())
                        connection.settimeout(0.25)
                        while not stop.is_set():
                            try:
                                latest = snapshots.get(timeout=0.2)
                                while True:
                                    latest = snapshots.get_nowait()
                            except Empty:
                                pass
                            if latest is not None:
                                _send(connection, latest)
                                latest = None
                    except (OSError, ConnectionError):
                        pass  # Disconnect affects this client only.
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        try:
            ready.send((False, str(exc)))
        except OSError:
            pass
    finally:
        ready.close()


class RemoteViewerServer:
    """Bounded sampler plus isolated sender; never waits for a physics lock."""

    def __init__(self, world, host='127.0.0.1', port=8765, rate_hz=15.0):
        if not 0 < rate_hz <= 60 or not 0 <= port <= 65535:
            raise ValueError('Viewer rate must be in (0, 60], port in [0, 65535]')
        self.world = world
        self.period = 1.0 / rate_hz
        self._state = np.empty(mujoco.mj_stateSize(world.model, SPEC))
        context = multiprocessing.get_context('spawn')
        self._snapshots = context.Queue(maxsize=1)
        self._stop = context.Event()
        parent, child = context.Pipe(duplex=False)
        self._process = context.Process(target=_serve,
            args=(world.model, self._snapshots, self._stop, (host, port), child),
            name='remote_viewer_sender', daemon=True)
        self._thread = None
        self._process.start()
        child.close()
        try:
            if not parent.poll(15.0):
                raise RuntimeError('Remote viewer sender startup timed out')
            ok, result = parent.recv()
            if not ok:
                raise RuntimeError(f'Remote viewer sender: {result}')
            self.port = result
        except Exception:
            self.close()
            raise
        finally:
            parent.close()
        self._thread = threading.Thread(target=self._sample, daemon=True,
                                        name='remote_viewer_sampler')
        self._thread.start()

    def _sample(self):
        while not self._stop.is_set() and self._process.is_alive():
            if self.world.lock.acquire(blocking=False):
                try:
                    mujoco.mj_getState(self.world.model, self.world.data, self._state, SPEC)
                finally:
                    self.world.lock.release()
                _put_latest(self._snapshots, self._state.astype('<f8', copy=False).tobytes())
            self._stop.wait(self.period)

    def close(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._process.join(timeout=1.0)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=1.0)
        self._snapshots.cancel_join_thread()
        self._snapshots.close()


def connect(host, port):
    sock = socket.create_connection((host, port), timeout=10.0)
    try:
        sock.settimeout(10.0)
        hello = json.loads(_receive(sock, 65536))
        if hello.get('protocol') != 1 or hello.get('mujoco') != mujoco.__version__:
            raise ValueError(f'Protocol/version mismatch: server {hello}; client MuJoCo {mujoco.__version__}')
        binary = _receive(sock)
        model = mujoco.MjModel.from_binary_path('remote.mjb', assets={'remote.mjb': binary})
        if hello['state_size'] != mujoco.mj_stateSize(model, SPEC):
            raise ValueError('Remote state size does not match model')
        return sock, model
    except Exception:
        sock.close()
        raise


def read_state(sock, model):
    count = mujoco.mj_stateSize(model, SPEC)
    packet = _receive(sock, count * 8)
    if len(packet) != count * 8:
        raise ValueError('Invalid remote state length')
    state = np.frombuffer(packet, dtype='<f8').astype(np.float64, copy=True)
    if not np.isfinite(state).all():
        raise ValueError('Non-finite remote state')
    return state


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--headless', action='store_true', help='Verify reception without opening a window')
    parser.add_argument('--frames', type=int, default=0, help='Exit after N frames (0: unlimited)')
    args = parser.parse_args(argv)
    if args.frames < 0 or not 1 <= args.port <= 65535:
        parser.error('frames must be nonnegative and port must be in [1, 65535]')
    sock, model = connect(args.host, args.port)
    data = mujoco.MjData(model)
    with sock:
        if args.headless:
            frames = 0
            while not args.frames or frames < args.frames:
                mujoco.mj_setState(model, data, read_state(sock, model), SPEC)
                mujoco.mj_forward(model, data)
                frames += 1
            print(f'Received {frames} frames; simulation time {data.time:.3f}s')
            return
        # Rendering never performs network reads; slow rendering drops old states.
        from mujoco import viewer as mjviewer
        from queue import Queue
        states = Queue(maxsize=1)
        done = threading.Event()

        def receive():
            try:
                while not done.is_set():
                    _put_latest(states, read_state(sock, model))
            except (OSError, ValueError, ConnectionError) as exc:
                if not done.is_set():
                    print(f'Remote viewer disconnected: {exc}', flush=True)
            finally:
                done.set()

        receiver = threading.Thread(target=receive, daemon=True)
        receiver.start()
        try:
            with mjviewer.launch_passive(model, data) as viewer:
                with viewer.lock():
                    viewer.cam.distance = 2.5
                    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'floating_base')
                    if body >= 0:
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                        viewer.cam.trackbodyid = body
                frames = 0
                while viewer.is_running() and not done.is_set():
                    try:
                        state = states.get_nowait()
                    except Empty:
                        time.sleep(0.005)
                        continue
                    with viewer.lock():
                        mujoco.mj_setState(model, data, state, SPEC)
                        mujoco.mj_forward(model, data)
                    viewer.sync()
                    frames += 1
                    if args.frames and frames >= args.frames:
                        break
                if args.frames and frames < args.frames:
                    raise RuntimeError(f'Viewer stopped after {frames}/{args.frames} requested frames')
                print(f'Rendered {frames} frames; simulation time {data.time:.3f}s')
        finally:
            done.set()
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            receiver.join(timeout=1.0)


if __name__ == '__main__':
    main()
