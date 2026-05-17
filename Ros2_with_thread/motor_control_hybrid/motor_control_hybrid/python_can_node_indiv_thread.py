#!/usr/bin/env python3
"""Compatibility wrapper for the direct-SDK Python CAN node."""

from motor_control_hybrid.python_can_node import PythonCanNode, main


__all__ = ["PythonCanNode", "main"]


if __name__ == "__main__":
    main()
