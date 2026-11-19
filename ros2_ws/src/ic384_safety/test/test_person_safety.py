"""Run with unittest; no ROS installation, model or hardware required."""
import importlib
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ic384_safety.person_safety_core import PersonStopPolicy, nearest_depth, pair_is_fresh


class DepthTests(unittest.TestCase):
    def test_millimetres_and_metres(self):
        self.assertEqual(nearest_depth([2000]*10, scale=.001), 2.0)
        self.assertEqual(nearest_depth([2.0]*10), 2.0)

    def test_nearest_foreground_is_conservative(self):
        self.assertEqual(nearest_depth([.7]+[3.0]*99), .7)

    def test_invalid_and_sparse_depth(self):
        for samples in ([], [0]*20, [math.nan]*20, [math.inf]*20,
                        [-1]*20, [.01]*20, [11]*20, [2.0]*8,
                        [2.0]*9+[0]*20):
            with self.subTest(samples=samples):
                self.assertIsNone(nearest_depth(samples))

    def test_bad_scale_is_rejected(self):
        for scale in (0, -1, math.nan):
            with self.assertRaises(ValueError):
                nearest_depth([1]*10, scale=scale)


class PolicyTests(unittest.TestCase):
    def setUp(self):
        self.policy = PersonStopPolicy()

    def test_near_person_and_hysteresis(self):
        self.assertTrue(self.policy.evaluate([2.0]).clear)
        self.assertFalse(self.policy.evaluate([1.0]).clear)
        self.assertFalse(self.policy.evaluate([1.2]).clear)
        self.assertTrue(self.policy.evaluate([1.5]).clear)
        self.assertTrue(self.policy.evaluate([1.2]).clear)

    def test_startup_in_hysteresis_band_remains_blocked(self):
        self.assertFalse(self.policy.evaluate([1.2]).clear)

    def test_all_people_must_be_clear(self):
        self.assertFalse(self.policy.evaluate([3.0, .8, 2.0]).clear)
        self.assertFalse(self.policy.evaluate([3.0, None]).clear)
        self.assertTrue(self.policy.evaluate([3.0, 2.0]).clear)

    def test_missing_depth_is_not_clear(self):
        for value in (None, math.nan, math.inf, 0, -1):
            self.assertFalse(self.policy.evaluate([value]).clear)

    def test_healthy_no_person_scene_can_clear(self):
        self.assertTrue(self.policy.evaluate([]).clear)

    def test_fault_blocks_until_release_distance(self):
        self.policy.evaluate([2.0])
        self.assertFalse(self.policy.fault('camera lost').clear)
        self.assertFalse(self.policy.evaluate([1.2]).clear)

    def test_bad_thresholds(self):
        for stop, release in ((0, 1), (2, 1), (1, 1), (1, math.inf), (math.nan, 2)):
            with self.assertRaises(ValueError):
                PersonStopPolicy(stop, release)


class FreshnessTests(unittest.TestCase):
    def fresh(self, stamps=(10.0, 10.01), received=(20.0, 20.01), now_ros=10.1, now_steady=20.1):
        return pair_is_fresh(stamps, received, now_ros, now_steady, .3, .05)

    def test_synchronized_pair(self):
        self.assertTrue(self.fresh())

    def test_recent_arrival_does_not_hide_stale_acquisition(self):
        self.assertFalse(self.fresh(stamps=(9.0, 9.01)))

    def test_unsynchronized_or_invalid_time(self):
        for stamps in ((10.0, 10.1), (11., 11.), (0., 0.), (math.nan, 10.)):
            self.assertFalse(self.fresh(stamps=stamps))

    def test_stalled_receipt_and_long_inference(self):
        self.assertFalse(self.fresh(now_steady=21.0))
        self.assertFalse(self.fresh(now_ros=10.5, now_steady=20.5))


class Message:
    def __init__(self, data=None):
        self.data = data


class Twist:
    def __init__(self):
        self.linear = SimpleNamespace(x=0., y=0., z=0.)
        self.angular = SimpleNamespace(x=0., y=0., z=0.)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeNode:
    require_person_clear = True
    require_hardware_feedback = True
    auto_resume_person = True

    def __init__(self, name):
        self.parameters = {}
        self.subscriptions = {}

    def declare_parameter(self, key, value):
        self.parameters[key] = getattr(self, key) if key in ('require_person_clear', 'require_hardware_feedback', 'auto_resume_person') else value

    def get_parameter(self, key):
        return SimpleNamespace(value=self.parameters[key])

    def create_publisher(self, *args):
        return Publisher()

    def create_subscription(self, msg_type, topic, callback, *args):
        self.subscriptions[topic] = callback

    def create_service(self, *args):
        pass

    def create_timer(self, *args):
        pass


def ros_stubs():
    definitions = {
        'rclpy': {}, 'rclpy.node': {'Node': FakeNode},
        'rclpy.qos': {'QoSProfile': object, 'ReliabilityPolicy': object, 'HistoryPolicy': object},
        'geometry_msgs': {}, 'geometry_msgs.msg': {'Twist': Twist},
        'std_msgs': {}, 'std_msgs.msg': {'Bool': Message, 'String': Message},
        'std_srvs': {}, 'std_srvs.srv': {'Trigger': object},
        'sensor_msgs': {}, 'sensor_msgs.msg': {'Image': object},
        'cv_bridge': {'CvBridge': object},
    }
    modules = {}
    for name, contents in definitions.items():
        modules[name] = ModuleType(name)
        modules[name].__dict__.update(contents)
    return modules


class SupervisorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.stub_context = patch.dict(sys.modules, ros_stubs())
        cls.stub_context.start()
        cls.module = importlib.import_module('ic384_safety.safety')

    @classmethod
    def tearDownClass(cls):
        sys.modules.pop('ic384_safety.safety', None)
        cls.stub_context.stop()

    def setUp(self):
        self.clock = patch.object(self.module.time, 'monotonic', return_value=100.0)
        self.now = self.clock.start()
        self.addCleanup(self.clock.stop)
        self.node = self.module.SafetySupervisor()

    def feedback(self, person=True):
        for key in self.node.input_keys:
            self.node.receive(key, Message(person if key == 'person_clear' else True))

    def reset(self):
        return self.node.reset(None, SimpleNamespace()).success

    def command(self):
        cmd = Twist()
        cmd.linear.x = .08
        self.node.receive_command(cmd)
        self.node.tick()

    def test_person_input_is_required_and_missing_blocks_reset(self):
        self.assertIn('/safety/input/person_clear', self.node.subscriptions)
        for key in ('estop_ok', 'obstacle_clear', 'arm_stowed'):
            self.node.receive(key, Message(True))
        self.assertFalse(self.reset())

    def test_person_stop_recovers_without_reset_but_requires_new_command(self):
        self.feedback()
        self.assertTrue(self.reset())
        self.command()
        self.assertEqual(self.node.pub.messages[-1].linear.x, .08)
        self.node.receive('person_clear', Message(False))
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)
        self.command()
        self.assertIsNone(self.node.command)
        self.feedback()
        self.node.tick()
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)
        self.assertTrue(self.node.armed)
        self.command()
        self.assertEqual(self.node.pub.messages[-1].linear.x, .08)

    def test_startup_still_requires_initial_reset(self):
        self.feedback()
        self.command()
        self.assertFalse(self.node.armed)
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)

    def test_estop_during_person_hold_remains_latched(self):
        self.feedback()
        self.assertTrue(self.reset())
        self.node.receive('person_clear', Message(False))
        self.node.receive('estop_ok', Message(False))
        self.feedback()
        self.command()
        self.assertFalse(self.node.armed)
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)
        self.assertTrue(self.reset())
        self.command()
        self.assertEqual(self.node.pub.messages[-1].linear.x, .08)

    def test_invalid_velocity_during_hold_still_latches(self):
        self.feedback()
        self.assertTrue(self.reset())
        self.node.receive('person_clear', Message(False))
        command = Twist()
        command.linear.x = math.nan
        self.node.receive_command(command)
        self.feedback()
        self.command()
        self.assertFalse(self.node.armed)
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)

    def test_other_feedback_timeout_during_hold_still_latches(self):
        self.feedback()
        self.assertTrue(self.reset())
        self.node.receive('person_clear', Message(False))
        self.now.return_value = 100.4
        self.node.tick()
        self.feedback()
        self.command()
        self.assertFalse(self.node.armed)
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)

    def test_velocity_timeout_outside_person_hold_still_latches(self):
        self.feedback()
        self.assertTrue(self.reset())
        self.command()
        self.now.return_value = 100.25
        self.feedback()
        self.node.tick()
        self.assertFalse(self.node.armed)
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)

    def test_manual_person_mode_can_still_be_selected(self):
        with patch.object(FakeNode, 'auto_resume_person', False):
            self.node = self.module.SafetySupervisor()
        self.feedback()
        self.assertTrue(self.reset())
        self.command()
        self.node.receive('person_clear', Message(False))
        self.feedback()
        self.command()
        self.assertFalse(self.node.armed)
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)
        self.assertTrue(self.reset())
        self.command()
        self.assertEqual(self.node.pub.messages[-1].linear.x, .08)

    def test_person_heartbeat_timeout_stops_and_recovers_on_fresh_clear(self):
        self.feedback()
        self.assertTrue(self.reset())
        self.command()
        self.now.return_value = 100.4
        for key in ('estop_ok', 'obstacle_clear', 'arm_stowed'):
            self.node.receive(key, Message(True))
        self.command()
        self.assertTrue(self.node.armed)
        self.assertTrue(self.node.person_hold)
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)
        self.assertIn('person_clear', self.node.fault())
        self.feedback()
        self.node.tick()
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)
        self.command()
        self.assertEqual(self.node.pub.messages[-1].linear.x, .08)

    def test_old_launch_does_not_require_new_input(self):
        with patch.object(FakeNode, 'require_person_clear', False):
            node = self.module.SafetySupervisor()
        self.assertNotIn('person_clear', node.input_keys)

    def camera_only(self):
        with patch.object(FakeNode, 'require_hardware_feedback', False):
            self.node = self.module.SafetySupervisor()

    def test_camera_only_has_no_hardware_subscriptions(self):
        self.camera_only()
        self.assertEqual(self.node.input_keys, ('person_clear',))
        for key in ('estop_ok', 'obstacle_clear', 'arm_stowed'):
            self.assertNotIn('/safety/input/'+key, self.node.subscriptions)
        self.feedback()
        self.assertTrue(self.reset())
        self.command()
        self.assertEqual(self.node.pub.messages[-1].linear.x, .08)

    def test_camera_only_missing_detector_blocks_start(self):
        self.camera_only()
        self.assertFalse(self.reset())
        self.command()
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)

    def test_camera_only_person_hold_and_automatic_recovery(self):
        self.camera_only()
        self.feedback()
        self.assertTrue(self.reset())
        self.command()
        self.node.receive('person_clear', Message(False))
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)
        self.command()
        self.feedback()
        self.node.tick()
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)
        self.command()
        self.assertEqual(self.node.pub.messages[-1].linear.x, .08)

    def test_camera_only_detector_timeout_stops(self):
        self.camera_only()
        self.feedback()
        self.assertTrue(self.reset())
        self.command()
        self.now.return_value = 100.4
        self.node.tick()
        self.assertTrue(self.node.person_hold)
        self.assertEqual(self.node.pub.messages[-1].linear.x, 0.)

    def test_all_safety_inputs_cannot_be_disabled(self):
        with patch.object(FakeNode, 'require_hardware_feedback', False), \
                patch.object(FakeNode, 'require_person_clear', False):
            with self.assertRaises(ValueError):
                self.module.SafetySupervisor()


class ImageArray:
    def __init__(self, shape, samples):
        self.shape, self.flat = shape, samples

    def __getitem__(self, key):
        return self


class CameraNodeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.stub_context = patch.dict(sys.modules, ros_stubs())
        cls.stub_context.start()
        cls.module = importlib.import_module('ic384_safety.person_safety')

    @classmethod
    def tearDownClass(cls):
        sys.modules.pop('ic384_safety.person_safety', None)
        cls.stub_context.stop()

    def setUp(self):
        self.clock = patch.object(self.module.time, 'monotonic', return_value=20.1)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.node = self.module.PersonSafety.__new__(self.module.PersonSafety)
        self.node.cfg = {'aligned_depth_confirmed': True, 'frame_timeout': .3,
                         'sync_tolerance': .05, 'depth_scale': .001,
                         'depth_min': .05, 'depth_max': 10., 'confidence': .35, 'device': 'cpu'}
        self.node.policy = PersonStopPolicy()
        self.node.clear_pub, self.node.status_pub = Publisher(), Publisher()
        self.node.last_pair = self.node.last_result = None
        header = SimpleNamespace(frame_id='rgb_optical', stamp=SimpleNamespace(sec=10, nanosec=0))
        self.rgb = SimpleNamespace(header=header, width=10, height=10, encoding='bgr8')
        self.depth = SimpleNamespace(header=header, width=10, height=10, encoding='16UC1')
        self.node.rgb, self.node.depth = (self.rgb, 20.), (self.depth, 20.)
        self.node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10100000000))
        self.depth_array = ImageArray((10, 10), [2000]*100)
        self.node.bridge = Mock()
        self.node.bridge.imgmsg_to_cv2.side_effect = lambda msg, fmt: (
            self.depth_array if msg is self.depth else ImageArray((10, 10, 3), []))
        self.boxes = Mock()
        self.boxes.xyxy.cpu.return_value.tolist.return_value = []
        self.node.model = Mock()
        self.node.model.predict.return_value = [SimpleNamespace(boxes=self.boxes)]
        self.node.person_ids = [0]

    def clear(self):
        return self.node.clear_pub.messages[-1].data

    def test_healthy_empty_scene(self):
        self.node.tick()
        self.assertTrue(self.clear())

    def test_near_person_stops(self):
        self.depth_array.flat = [800]*100
        self.boxes.xyxy.cpu.return_value.tolist.return_value = [[1., 1., 9., 9.]]
        self.node.tick()
        self.assertFalse(self.clear())

    def test_duplicate_pair_does_not_refresh_clearance(self):
        self.node.tick()
        self.node.tick()
        self.assertEqual(len(self.node.clear_pub.messages), 1)
        self.node.model.predict.assert_called_once()

    def test_expired_inference_cannot_publish_clear(self):
        with patch.object(self.node, 'fresh', side_effect=[True, False]):
            self.node.tick()
        self.assertFalse(self.clear())

    def test_detector_exception_stops(self):
        self.node.model.predict.side_effect = RuntimeError('inference failed')
        self.node.tick()
        self.assertFalse(self.clear())

    def test_bad_depth_blocks_even_when_no_person_detected(self):
        self.depth_array.flat = [0]*100
        self.node.tick()
        self.assertFalse(self.clear())
        self.node.model.predict.assert_not_called()

    def test_unconfirmed_alignment_blocks(self):
        self.node.cfg['aligned_depth_confirmed'] = False
        self.node.tick()
        self.assertFalse(self.clear())

    def test_missing_frames_blocks(self):
        self.node.depth = None
        self.node.tick()
        self.assertFalse(self.clear())

    def test_frame_mismatch_blocks(self):
        self.depth.header = SimpleNamespace(frame_id='different_frame')
        self.node.tick()
        self.assertFalse(self.clear())

    def test_invalid_detection_output_blocks(self):
        self.node.model.predict.return_value = []
        self.node.tick()
        self.assertFalse(self.clear())

    def test_metres_depth_encoding(self):
        self.depth.encoding = '32FC1'
        self.depth_array.flat = [.8]*100
        self.boxes.xyxy.cpu.return_value.tolist.return_value = [[1., 1., 9., 9.]]
        self.node.tick()
        self.assertFalse(self.clear())


if __name__ == '__main__':
    unittest.main()
