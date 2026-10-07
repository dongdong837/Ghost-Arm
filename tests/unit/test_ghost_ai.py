import json,unittest
from unittest.mock import patch
from ghost_sim.ai import ghost_ai as ai
from ghost_sim.navigation.planner3d import Planner

class GhostAITests(unittest.TestCase):
    def test_relative_and_absolute_height(self):
        p=[1,0,1.2,.7]
        self.assertEqual(ai.resolve({'action':'relative','delta':[0,0,.5]},p,{})['position'],[1,0,1.7])
        self.assertEqual(ai.resolve({'action':'height','z':2.0},p,{})['position'],[1,0,2.0])
        self.assertEqual(ai.resolve({'action':'height','z':2.0},p,{})['yaw'],.7)
    def test_above_landmark_is_flyable(self):
        places=ai.landmarks();planner=Planner()
        for target in ['home','green_box','orange_box','green_box_above','orange_box_above']:
            action=ai.resolve({'action':'navigate','target':target},[0,0,1.2,0],places)
            self.assertTrue(planner.free(action['position']),target)
    def test_strict_schema(self):
        for item in ['[]','bad','{"action":"run","shell":"echo bad"}',
                     '{"action":"navigate","position":[0,0,1],"target":"home"}',
                     '{"action":"stop","extra":1}']:
            with self.assertRaises(ValueError):ai.parse(item)
    def test_reject_non_numeric_position_and_unknown_target(self):
        for action in [{'action':'navigate','position':[True,0,1]},
                       {'action':'relative','delta':[0,0,float('nan')]},
                       {'action':'navigate','target':[]}]:
            with self.assertRaises(ValueError):ai.resolve(action,[0,0,1.2,0],ai.landmarks())
    def test_stop_without_api_or_pose(self):
        with patch.object(ai.sys,'argv',['ghost_ai.py','停下','--dry-run']),patch.object(ai,'call_model') as model,patch.object(ai,'current_pose') as pose:
            self.assertEqual(ai.main(),0);model.assert_not_called();pose.assert_not_called()
    def test_dry_run_does_not_move(self):
        with patch.object(ai.sys,'argv',['ghost_ai.py','升高半米','--dry-run']),patch.dict(ai.os.environ,{'STEPFUN_API_KEY':'test'}),patch.object(ai,'call_model',return_value='{"action":"relative","delta":[0,0,0.5]}'),patch.object(ai,'current_pose',return_value=[0,0,1.2,0]),patch.object(ai.subprocess,'run') as run:
            self.assertEqual(ai.main(),0);run.assert_not_called()
    def test_vision_rejects_motion(self):
        with patch.object(ai.sys,'argv',['ghost_ai.py','观察','--vision']),patch.dict(ai.os.environ,{'STEPFUN_API_KEY':'test'}),patch.object(ai,'capture_camera',return_value='image'),patch.object(ai,'call_model',return_value='{"action":"height","z":2}'),patch.object(ai.subprocess,'run') as run:
            self.assertEqual(ai.main(),1);run.assert_not_called()

if __name__=='__main__':unittest.main()
