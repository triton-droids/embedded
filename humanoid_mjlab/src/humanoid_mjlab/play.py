"""Play a checkpoint with mjlab's play script, or check the env with zero/random actions.

mjlab/.venv/bin/python -m humanoid_mjlab.play Mjlab-Velocity-Flat-Humanoid --checkpoint-file latest
mjlab/.venv/bin/python -m humanoid_mjlab.play Mjlab-Velocity-Flat-Humanoid --agent zero
"""

from mjlab.scripts.play import main

import humanoid_mjlab.tasks  # noqa: F401  (registers the task)

if __name__ == "__main__":
    main()
