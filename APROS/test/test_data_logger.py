import configparser
import csv
import json
import os
import struct
import tempfile
import threading
import unittest
from types import SimpleNamespace

from core.data_logger import DataLogger


class Device(SimpleNamespace):
    def add_nmea_listener(self, listener):
        self.listener = listener

    def remove_nmea_listener(self, listener):
        if self.listener is listener:
            self.listener = None


class DataLoggerTest(unittest.TestCase):
    def test_slow_writer_does_not_block_other_device_and_stop_drains(self):
        robot = SimpleNamespace(config=configparser.ConfigParser(), devices={})
        with tempfile.TemporaryDirectory() as directory:
            log = DataLogger(robot, directory)
            log.start_recording()
            blocked = threading.Event()
            release = threading.Event()
            other_written = threading.Event()
            def slow(timestamp, value):
                blocked.set()
                release.wait(2)
            def fast(timestamp, value):
                other_written.set()
            first = log._make_recorder(Device(name='slow'), slow)
            second = log._make_recorder(Device(name='fast'), fast)
            first('one')
            self.assertTrue(blocked.wait(1))
            first('two')
            second('other')
            self.assertTrue(other_written.wait(1))
            stopper = threading.Thread(target=log.stop_recording)
            stopper.start()
            stopper.join(0.05)
            self.assertTrue(stopper.is_alive())
            release.set()
            stopper.join(2)
            self.assertFalse(stopper.is_alive())

    def test_records_receive_callbacks_and_stops(self):
        names = ['vlp-16', 'ouster-sr-128', 'basler_gige_camera',
                 'synerex_syn3100', 'fluke_sv600', 'xsens_mti_630r', 'mobile_drive_s1']
        devices = {name: Device(name=name, enable=True) for name in names}
        devices['vlp-16'].port = 2368
        devices['ouster-sr-128'].port = 7502
        devices['ouster-sr-128'].bind_ip = '192.168.10.2'
        devices['ouster-sr-128']._sensor_info = SimpleNamespace(to_json_string=lambda: '{"sensor":1}')
        devices['mobile_drive_s1'].parser = SimpleNamespace(parse=lambda i, d: {'vehicle_velocity_val': 2.0, 'vehicle_steer_angle_val': -3.0})
        robot = SimpleNamespace(config=configparser.ConfigParser(), devices=devices)
        with tempfile.TemporaryDirectory() as directory:
            log = DataLogger(robot, directory)
            log.start_recording()
            session = log.session_dir
            devices['vlp-16'].data_logger_packet(b'vlp', ('192.168.1.1', 2368))
            devices['ouster-sr-128'].data_logger_packet(b'os', ('192.168.1.2', 7502))
            for name in ('basler_gige_camera', 'fluke_sv600'):
                devices[name].data_logger_frame(b'jpeg1', {'frame_id': 1})
                devices[name].data_logger_frame(b'jpeg2', {'frame_id': 2})
            devices['synerex_syn3100'].listener('$GNGGA,1')
            devices['xsens_mti_630r'].data_logger_raw(b'\xfa\x01', None)
            devices['mobile_drive_s1'].frame_recorder(SimpleNamespace(id=0x304, data=b'123456'))
            old_callback = devices['vlp-16'].data_logger_packet
            log.stop_recording()
            old_callback(b'after stop', ('192.168.1.1', 2368))
            with open(os.path.join(session, 'vlp-16.pcap'), 'rb') as f:
                contents = f.read()
            self.assertEqual(len(contents), 24 + 16 + 42 + 3)
            self.assertEqual(struct.unpack('<I', contents[32:36])[0], 45)
            self.assertEqual(len(os.listdir(os.path.join(session, 'basler_gige_camera'))), 2)
            self.assertEqual(len(os.listdir(os.path.join(session, 'fluke_sv600'))), 2)
            with open(os.path.join(session, 'synerex_syn3100.nmea')) as f:
                self.assertIn('$GNGGA,1', f.read())
            with open(os.path.join(session, 'xsens_mti_630r.jsonl')) as f:
                self.assertEqual(json.loads(f.readline())['data_hex'], 'fa01')
            with open(os.path.join(session, 'mobile_can.csv')) as f:
                row = list(csv.DictReader(f))[0]
            self.assertEqual((row['id'], row['speed_kmh'], row['steer_angle_deg']), ('0x304', '2.0', '-3.0'))
            self.assertIsNone(devices['synerex_syn3100'].listener)
            log.start_recording()
            self.assertNotEqual(session, log.session_dir)
            log.stop_recording()


if __name__ == '__main__':
    unittest.main()
