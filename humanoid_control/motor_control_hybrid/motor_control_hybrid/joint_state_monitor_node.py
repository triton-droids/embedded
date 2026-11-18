#!/usr/bin/env python3
"""Compatibility entry point; the read-only RViz relay now lives in safety."""
from humanoid_safety.joint_state_monitor_node import (  # noqa: F401
    JointStateMonitorNode, main, valid_joint_state)

if __name__ == '__main__':
    main()
