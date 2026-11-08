"""Real TCP/MJB transfer without ROS or a display."""
import json
import socket
import struct
import time

import mujoco
import numpy as np
import pytest

from humanoid_sim.remote_viewer import (RemoteViewerServer, SPEC, _receive,
                                       connect, read_state)
from test_world import make_world


def test_remote_model_assets_state_and_reconnect():
    world = make_world()
    server = RemoteViewerServer(world, port=0, rate_hz=60)
    try:
        for _ in range(2):
            with connect('127.0.0.1', server.port)[0] as connection:
                # Reconnect to the same sender after closing this session.
                state = read_state(connection, world.model)
                assert state.size == mujoco.mj_stateSize(world.model, SPEC)
            time.sleep(0.1)
        connection, model = connect('127.0.0.1', server.port)
        with connection:
            assert model.nmesh == world.model.nmesh
            np.testing.assert_array_equal(model.mesh_vert, world.model.mesh_vert)
            world.step(0.0)
            data = mujoco.MjData(model)
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                mujoco.mj_setState(model, data, read_state(connection, model), SPEC)
                if data.time == world.data.time:
                    break
            assert data.time == world.data.time
            mujoco.mj_forward(model, data)
            np.testing.assert_allclose(data.qpos, world.data.qpos)
            np.testing.assert_allclose(data.xpos, world.data.xpos, atol=1e-3)
    finally:
        server.close()
    assert not server._process.is_alive()
    assert not server._thread.is_alive()


def test_absent_or_idle_viewer_does_not_stop_physics():
    world = make_world()
    server = RemoteViewerServer(world, port=0, rate_hz=60)
    world.start()
    try:
        time.sleep(0.1)
        before = world.steps
        connection, _ = connect('127.0.0.1', server.port)
        with connection:
            # Deliberately never consume state packets.
            time.sleep(0.2)
            assert world.steps > before
        before = world.steps
        time.sleep(0.1)
        assert world.steps > before
    finally:
        world.stop()
        server.close()


def test_occupied_port_reports_startup_error():
    with socket.socket() as listener:
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        with pytest.raises(RuntimeError, match='Remote viewer sender'):
            RemoteViewerServer(make_world(), port=listener.getsockname()[1])


def test_frame_limit_rejects_oversize_before_payload():
    left, right = socket.socketpair()
    with left, right:
        left.sendall(struct.pack('!I', 1000))
        with pytest.raises(ValueError, match='exceeds limit'):
            _receive(right, 8)


def test_version_mismatch_is_rejected_before_loading_model(monkeypatch):
    left, right = socket.socketpair()
    with left:
        hello = json.dumps(dict(protocol=1, mujoco='incompatible')).encode()
        left.sendall(struct.pack('!I', len(hello)) + hello)
        monkeypatch.setattr(socket, 'create_connection', lambda *a, **kw: right)
        with pytest.raises(ValueError, match='version mismatch'):
            connect('unused', 8765)
        assert right.fileno() == -1


def test_nonfinite_state_is_rejected():
    model = make_world().model
    state = np.zeros(mujoco.mj_stateSize(model, SPEC), dtype='<f8')
    state[0] = np.nan
    packet = state.tobytes()
    left, right = socket.socketpair()
    with left, right:
        left.sendall(struct.pack('!I', len(packet)) + packet)
        with pytest.raises(ValueError, match='Non-finite'):
            read_state(right, model)


@pytest.mark.parametrize('rate', [0, -1, 61, float('nan')])
def test_invalid_rate_is_rejected(rate):
    with pytest.raises(ValueError):
        RemoteViewerServer(make_world(), rate_hz=rate)
