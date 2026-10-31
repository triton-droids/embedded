from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "humanoid_control/wbc_control_node"))

from wbc_control_node.controller import WBCController

__all__ = ["WBCController"]
