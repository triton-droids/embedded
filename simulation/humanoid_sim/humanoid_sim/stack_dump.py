"""SIGUSR1 prints every thread's Python stack to stderr (no tooling needed).

``kill -USR1 <pid>`` on a stalled simulator process shows where each thread
waits; registering costs nothing until the signal arrives.
"""
import faulthandler
import signal


def enable() -> None:
    try:
        faulthandler.register(signal.SIGUSR1, all_threads=True, chain=False)
    except (AttributeError, ValueError, RuntimeError):  # Not available on this platform.
        pass
