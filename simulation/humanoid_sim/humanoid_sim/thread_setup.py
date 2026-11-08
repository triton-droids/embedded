"""Opt-in experiment hook; ordinary simulation has no scheduling changes."""
import importlib
import os


def configure_thread(role):
    module = os.environ.get('HUMANOID_THREAD_SCHEDULER', '')
    if module:
        return importlib.import_module(module).configure_current_thread(role)
    return None
