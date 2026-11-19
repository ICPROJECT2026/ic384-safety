from array import array
import io
import struct
import unittest

from check_depth import DepthFrame, HEADER, read_frame, summarize


class BridgeTests(unittest.TestCase):
    def encoded(self, width=4, height=3, payload=None, magic=b'ADP1'):
        payload = struct.pack('<12H', *([1200]*12)) if payload is None else payload
        return io.BytesIO(HEADER.pack(magic, width, height, 7, 123456, 1000000000)+payload)

    def test_frame_roundtrip_millimetres(self):
        frame = read_frame(self.encoded())
        self.assertEqual((frame.width, frame.height, frame.index), (4, 3, 7))
        self.assertEqual(list(frame.millimetres), [1200]*12)

    def test_truncated_data_is_rejected(self):
        with self.assertRaises(EOFError):
            read_frame(self.encoded(payload=b'\0\0'))

    def test_wrong_header_is_rejected(self):
        for kwargs in ({'magic': b'BAD!'}, {'width': 0}, {'width': 4097}):
            with self.assertRaises(ValueError):
                read_frame(self.encoded(**kwargs))

    def test_stop_and_release_boundaries_are_diagnostic_only(self):
        for depth, zone in ((1000, 'DEPTH_WITHIN_STOP_DISTANCE'),
                            (1200, 'DEPTH_BETWEEN_THRESHOLDS'),
                            (1500, 'DEPTH_BEYOND_RELEASE_DISTANCE')):
            frame = DepthFrame(4, 3, 1, 100, 1000000000, array('H', [depth]*12))
            report = summarize(frame, now_ns=1100000000)
            self.assertEqual(report['depth_zone'], zone)
            self.assertFalse(report['motion_allowed'])
            self.assertFalse(report['person_distance_verified'])

    def test_stale_and_future_frames_are_unknown(self):
        frame = read_frame(self.encoded())
        for now_ns in (2000000000, 900000000):
            report = summarize(frame, now_ns=now_ns)
            self.assertFalse(report['depth_valid'])
            self.assertEqual(report['depth_zone'], 'UNKNOWN')

    def test_empty_depth_is_not_clear(self):
        frame = DepthFrame(4, 3, 1, 100, 1000000000, array('H', [0]*12))
        self.assertFalse(summarize(frame, now_ns=1100000000)['depth_valid'])


if __name__ == '__main__':
    unittest.main()
