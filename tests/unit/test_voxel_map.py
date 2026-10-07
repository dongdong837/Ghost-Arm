"""Validate height separation and conservative clearance for entire free voxels."""
from ghost_sim.paths import REPORTS
import itertools
import json
import unittest
import numpy as np
from ghost_sim.mapping.voxel_map import save,query,FREE,OCCUPIED,CLEARANCE

class VoxelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.states,cls.centres,cls.meta=save()

    def test_shape_and_labels(self):
        self.assertEqual(self.states.shape,(80,60,30))
        self.assertEqual(set(np.unique(self.states)),{FREE,CLEARANCE,OCCUPIED})

    def test_height_changes_flyability(self):
        self.assertEqual(query(-1.8,1.2,.4)['state'],'physical_obstacle')
        self.assertEqual(query(-1.8,1.2,1.0)['state'],'clearance_exclusion')
        self.assertTrue(query(-1.8,1.2,1.5)['flyable'])
        self.assertTrue(query(0,0,1.2)['flyable'])
        self.assertEqual(query(0,0,1.2)['voxel_xyz'][2],12)

    def test_boundaries(self):
        for point in [(4,0,1),(-4.01,0,1),(0,0,3),(0,0,-.1)]:
            self.assertEqual(query(*point)['state'],'outside_map')
        for point in [(0,0,.2),(0,0,2.9),(3.8,0,1)]:
            self.assertFalse(query(*point)['flyable'])

    def test_all_free_voxel_corners_have_clearance(self):
        centres=self.centres[self.states==FREE]
        minimum=float('inf')
        for corner in itertools.product([-.05,.05],repeat=3):
            points=centres+corner
            self.assertTrue(np.all(points>=np.array(self.meta['origin'])+.33-1e-8))
            self.assertTrue(np.all(points<=np.array(self.meta['upper_exclusive'])-.33+1e-8))
            for box in self.meta['boxes']:
                distances=np.linalg.norm(np.maximum(np.abs(points-box['centre'])-box['half_size'],0),axis=1)
                minimum=min(minimum,float(distances.min()))
        self.assertGreater(minimum,.33-1e-8)
        report={'result':'PASS','shape':list(self.states.shape),'counts':self.meta['counts'],
                'minimum_free_corner_clearance_m':minimum,'inflation_radius_m':.33,
                'height_examples':{str(z):query(-1.8,1.2,z) for z in [.4,1.,1.5]}}
        (REPORTS/'voxel_validation.json').write_text(json.dumps(report,indent=2)+'\n')

if __name__=='__main__':unittest.main()
