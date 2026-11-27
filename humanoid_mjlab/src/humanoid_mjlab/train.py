"""Train the humanoid with mjlab's PPO training script.

mjlab/.venv/bin/python -m humanoid_mjlab.train Mjlab-Velocity-Flat-Humanoid --env.scene.num-envs 4096
"""

from mjlab.scripts.train import main

import humanoid_mjlab.tasks  # noqa: F401  (registers the task)

if __name__ == "__main__":
    main()
