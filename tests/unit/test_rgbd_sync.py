import unittest
from ghost_sim.perception.rgbd_sync import RGBDSync

class RGBDSyncTests(unittest.TestCase):
    def test_restart_with_full_old_cache(self):
        sync=RGBDSync(3)
        for stamp in [100,101,102]:
            for kind in ['rgb','depth','info']:sync.push(kind,stamp,(kind,stamp))
        for kind in ['rgb','depth','info']:sync.push(kind,1,(kind,1))
        self.assertEqual(sync.latest()['depth'],('depth',1))
        self.assertGreater(sync.resets,0)

    def test_only_same_stamp_matches(self):
        sync=RGBDSync()
        sync.push('rgb',1,'rgb');sync.push('depth',2,'depth');sync.push('info',2,'info')
        self.assertIsNone(sync.latest())
        sync.push('rgb',2,'new')
        self.assertEqual(sync.latest()['rgb'],'new')
        self.assertIsNone(sync.latest())

    def test_missing_stream_bounded(self):
        sync=RGBDSync(3)
        for stamp in range(100):sync.push('rgb',stamp,stamp)
        self.assertEqual(len(sync.frames),3)
