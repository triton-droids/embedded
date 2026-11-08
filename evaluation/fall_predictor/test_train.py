"""Check separation, threshold selection, ranking ties and model serialization."""
import tempfile
from pathlib import Path
import unittest

import numpy as np
import torch

from train import FallMLP, ranking_metrics, recall_threshold, stratified_episode_split


class TrainingChecks(unittest.TestCase):
    def test_whole_episode_separation_is_deterministic(self):
        episodes=[dict(index=i,mode=str(i%2),fall_time=1. if i%3 else None,
                       actual_push=bool(i%5)) for i in range(200)]
        a=stratified_episode_split(episodes,7)
        b=stratified_episode_split(episodes,7)
        self.assertEqual(a,b)
        self.assertEqual(set.union(*(set(v) for v in a.values())),set(range(200)))
        self.assertFalse(set(a['train']) & set(a['validation']))
        self.assertFalse(set(a['train']) & set(a['test']))
        self.assertFalse(set(a['validation']) & set(a['test']))

    def test_legacy_float_boundary_is_corrected_without_changing_raw_file(self):
        import json
        from train import load_data, FEATURES
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);ep=root/'episode_000000';ep.mkdir()
            fall=.7900000000000469
            rows=[]
            for i in range(120):
                t=i*.01
                label=None if i<19 or t>=fall else int(t<fall<=t+.3)
                rows.append(dict(sim_time_s=t,features=[0.]*8,base_held=False,
                    control_fault=None,estop=False,label=label))
            text=''.join(json.dumps(r)+'\n' for r in rows)
            (ep/'samples.jsonl').write_text(text)
            (ep/'metadata.json').write_text(json.dumps(dict(feature_names=FEATURES,
                history_frames=20,fall_time_sim_s=fall,push_applied_steps=0,push_force_world_n=[0,0,0])))
            (root/'episodes.jsonl').write_text(json.dumps(dict(valid=True,episode=0,mode='test',seed=1))+'\n')
            loaded=load_data(root)
            self.assertGreater(loaded[-1].get('corrected_300ms_boundary_labels',0),0)
            self.assertEqual((ep/'samples.jsonl').read_text(),text)

    def test_tied_rank_scores_equal_chance(self):
        r=ranking_metrics(np.array([0,1,0,1]),np.ones(4))
        self.assertAlmostEqual(r['average_precision'],.5)
        self.assertAlmostEqual(r['auroc'],.5)

    def test_threshold_meets_recall_without_using_negative_labels(self):
        scores=np.arange(100,dtype=float)/100
        y=np.ones(100,dtype=int)
        threshold=recall_threshold(y,scores,.95)
        self.assertEqual(threshold,.05)
        self.assertGreaterEqual(np.mean(scores>=threshold),.95)

    def test_export_includes_normalization_and_matches_eager(self):
        model=FallMLP(np.arange(8,dtype=np.float32),np.arange(1,9,dtype=np.float32)).eval()
        x=torch.randn(3,20,8)
        traced=torch.jit.trace(model,torch.zeros(1,20,8))
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'model.pt';traced.save(str(path))
            restored=torch.jit.load(str(path))
            torch.testing.assert_close(model(x),restored(x))
        self.assertEqual(sum(p.numel() for p in model.parameters()),12417)


if __name__=='__main__': unittest.main()
