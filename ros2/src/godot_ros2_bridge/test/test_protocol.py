import copy
import json
import sys
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from godot_ros2_bridge.transport import JsonLines,encode,valid_state,MAX_LINE

class ProtocolTests(unittest.TestCase):
    def test_fragmented_utf8_and_coalesced_frames(self):
        parser=JsonLines()
        first=encode({'type':'hello','description':'巡检'})
        second=encode({'type':'ping'})
        self.assertEqual(parser.feed(first[:7]),[])
        packets=parser.feed(first[7:]+second)
        self.assertEqual([p['type'] for p in packets],['hello','ping'])
        self.assertEqual(packets[0]['description'],'巡检')

    def test_oversized_line_rejected(self):
        with self.assertRaises(ValueError):
            JsonLines().feed(b'x'*(MAX_LINE+1))

    def test_non_object_rejected(self):
        with self.assertRaises(ValueError):
            JsonLines().feed(b'[]\n')

    def test_nonfinite_encoding_rejected(self):
        with self.assertRaises(ValueError):
            encode({'linear_x':float('nan')})

    def test_real_simulator_fixture(self):
        fixture=json.loads((Path(__file__).parent/'fixture_state.json').read_text(encoding='utf-8'))
        self.assertTrue(valid_state(fixture))
        damaged=copy.deepcopy(fixture)
        damaged['pose']['orientation']=[0,0,0,0]
        self.assertFalse(valid_state(damaged))
        damaged=copy.deepcopy(fixture)
        damaged['imu']['linear_acceleration'][0]=float('inf')
        self.assertFalse(valid_state(damaged))
        damaged=copy.deepcopy(fixture)
        damaged['laser']['ranges']=[-1.]
        self.assertFalse(valid_state(damaged))

if __name__=='__main__':
    unittest.main()
