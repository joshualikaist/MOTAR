import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
import torch

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'tools'))
from perception_temporal import build_temporal_model
from perception_candidates import sha256_file
from perception_streaming import StreamingSelector
from perception_crop_verifier import crop_tensor, CropVerifier


class StreamingTest(unittest.TestCase):
    def test_sequence_reset_timestamp_guard_and_empty_frame(self):
        config=json.loads((ROOT/'configs/perception_candidate_motion_transformer_v1.json').read_text())
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'model.pt'
            torch.save({'config':config,'model_state_dict':build_temporal_model(config).state_dict()},path)
            with self.assertRaisesRegex(ValueError,'hash'):
                StreamingSelector(path,'0'*64,'cpu')
            selector=StreamingSelector(path,sha256_file(path),'cpu')
            zero=np.zeros((5,12),np.float32)
            result=selector.step([],'a',0,100,100,zero)
            self.assertIsNone(result['predicted_rank'])
            with self.assertRaisesRegex(ValueError,'timestamp'):
                selector.step([],'a',0,100,100,zero)
            with self.assertRaisesRegex(ValueError,'geometry'):
                selector.step([],'a',1,101,100,zero)
            selector.step([],'b',0,200,100,zero)
            self.assertEqual(len(selector.frames),1)

    def test_crop_rgb_channel_order_and_feature_norm(self):
        frame=np.zeros((40,40,3),np.uint8);frame[:,:,2]=255
        c={'rank':0,'u_px':20.,'v_px':20.,'width_px':10.,'height_px':10.}
        crops=crop_tensor(frame,[c])
        self.assertTrue((crops[0,0]==255).all())
        self.assertTrue((crops[0,2]==0).all())
        scores,features=CropVerifier()(torch.from_numpy(crops[:1]))
        self.assertEqual(tuple(scores.shape),(1,))
        self.assertAlmostEqual(float(features.norm()),1.,places=5)


if __name__=='__main__':unittest.main()
