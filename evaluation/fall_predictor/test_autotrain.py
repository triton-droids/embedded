import unittest
import numpy as np
import torch
from autotrain import Candidate, alarm_mask, policy_key


class AutoTrainingChecks(unittest.TestCase):
    def test_confirmation_resets_on_gap_and_episode(self):
        mask=alarm_mask(np.ones(7),np.array([0,0,0,0,1,1,1]),
                        np.array([0,.01,.02,.06,0,.01,.02]),.5,3)
        np.testing.assert_array_equal(mask,[False,False,True,False,False,False,True])

    def test_recall_gate_precedes_lower_false_alarm_rate(self):
        good=dict(qualifies=True,false_alarm_onsets_per_hour=20,event_recall=.96,
                  median_before_safety_estop_ms=60)
        bad=dict(qualifies=False,false_alarm_onsets_per_hour=0,event_recall=.90,
                 median_before_safety_estop_ms=100)
        self.assertGreater(policy_key(good),policy_key(bad))

    def test_all_families_export_same_batch_output(self):
        for family in ['small','base','wide','cnn']:
            model=Candidate(np.zeros(8,dtype=np.float32),np.ones(8,dtype=np.float32),family,.1).eval()
            x=torch.randn(3,20,8)
            trace=torch.jit.trace(model,torch.zeros(1,20,8))
            torch.testing.assert_close(model(x),trace(x))
            self.assertEqual(model(x).shape,(3,))


if __name__=='__main__':
    torch.set_num_threads(3)
    unittest.main()
