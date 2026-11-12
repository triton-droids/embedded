import mujoco
import numpy as np

from humanoid_sim.state_shm import StateReader, StateWriter

XML = """<mujoco><worldbody>
  <body name="b"><freejoint/><geom size="0.1"/></body>
  <body name="m" mocap="true"><geom size="0.1" contype="0" conaffinity="0"/></body>
</worldbody></mujoco>"""


def test_round_trip_and_model(tmp_path):
    model = mujoco.MjModel.from_xml_string(XML)
    writer = StateWriter(model, str(tmp_path))
    reader = StateReader(str(tmp_path), timeout_s=1.0)
    assert reader.read() is None  # Nothing written yet.
    qpos = np.arange(model.nq, dtype=float)
    writer.write(123, qpos, [[1., 2., 3.]], [[1., 0., 0., 0.]])
    stamp, q, mp, mq = reader.read()
    assert stamp == 123
    assert np.array_equal(q, qpos)
    assert np.array_equal(mp, [[1., 2., 3.]]) and np.array_equal(mq, [[1., 0., 0., 0.]])
    loaded = mujoco.MjModel.from_binary_path(reader.model_path)
    assert loaded.nq == model.nq and loaded.nmocap == model.nmocap
    reader.close()
    writer.close()
