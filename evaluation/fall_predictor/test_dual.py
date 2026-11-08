import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np
import torch

from train_dual import DualRisk, scenario_folds, recovery_scenario
from train import FEATURES


class DualChecks(unittest.TestCase):
    def test_family_splits_do_not_overlap_and_pilots_are_train(self):
        folds=scenario_folds()
        self.assertEqual(len(folds),48)
        self.assertEqual([sum(v==k for v in folds.values()) for k in ['train','validation','test']],[32,8,8])
        for seed in [48000,48001,48002]:self.assertEqual(folds[recovery_scenario(seed)['family']],'train')

    def test_nested_heads_and_export(self):
        model=DualRisk(np.zeros(8),np.ones(8)).eval();x=torch.randn(9,20,8)
        a=model(x);self.assertTrue(torch.all(a[:,0]<=a[:,1]))
        export=torch.jit.trace(model,torch.zeros(1,20,8))
        torch.testing.assert_close(a,export(x))
        loss=torch.nn.functional.binary_cross_entropy_with_logits(a,torch.zeros_like(a))
        loss.backward()
        self.assertTrue(all(torch.isfinite(p.grad).all() for p in model.parameters() if p.grad is not None))

    def test_full_pipeline_on_synthetic_fixture(self):
        # Fixtures validate orchestration/serialization, not physical prediction quality.
        with tempfile.TemporaryDirectory(prefix='dual-training-test-') as directory:
            root=Path(directory);old=root/'old';new=root/'new';old.mkdir();new.mkdir()
            (old/'episodes.jsonl').write_text('')
            split=root/'old_splits.json';split.write_text(json.dumps(dict(train=[],validation=[],test=[])))
            manifest=[]
            for family_index in range(48):
                for fallen in [False,True]:
                    index=family_index*2+int(fallen);p=new/f'episode_{index:06d}';p.mkdir()
                    fall=2. if fallen else None
                    metadata=dict(feature_names=FEATURES,fall_time_sim_s=fall,
                                  recovered_after_push=not fallen,scenario=recovery_scenario(family_index))
                    (p/'metadata.json').write_text(json.dumps(metadata))
                    rows=[]
                    for i in range(250):
                        t=i*.01;tilt=max(0,t-1.)*.6 if fallen else .02
                        rows.append(dict(sim_time_s=t,features=[tilt,0.,tilt*2,0.,0.,0.,0.,9.81],
                            base_held=False,control_fault=None,estop=fallen and t>=1.95,
                            label=None if fall is not None and t>=fall else int(fall is not None and 0<fall-t<=.3)))
                    (p/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
                    manifest.append(dict(valid=True,episode=index))
            (new/'episodes.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in manifest))
            out=root/'model'
            command=[sys.executable,str(Path(__file__).with_name('train_dual.py')),
                '--legacy-source',str(old),'--legacy-splits',str(split),'--new-source',str(new),
                '--output',str(out),'--epochs','1','--max-trials','1']
            result=subprocess.run(command,capture_output=True,text=True,timeout=120)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            status=json.loads((out/'status.json').read_text())
            self.assertEqual(status['state'],'complete');self.assertEqual(status['test_evaluations'],1)
            best=json.loads((out/'best.json').read_text())
            model=torch.jit.load(best['model_path'])
            logits=model(torch.zeros(1,20,8));self.assertTrue(torch.all(logits[:,0]<=logits[:,1]))
            self.assertTrue((out/'sealed_test_metrics.json').exists())


if __name__=='__main__':
    torch.set_num_threads(3)
    unittest.main()
