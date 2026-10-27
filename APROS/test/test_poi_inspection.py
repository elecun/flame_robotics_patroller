import unittest
from types import SimpleNamespace
from unittest.mock import patch

from core.plugin.mission_manager import MissionManager


class FakeClock:
    def __init__(self):
        self.now = 100.0

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class FakeDes:
    is_connected = True

    def __init__(self):
        self.height = 2900.0
        self.events = []
        self.completed = True

    def acquire_control(self):
        pass

    def get_mast_height(self):
        return self.height

    def set_mast_extend(self, target):
        self.events.append(('extend', target))
        self.height = target - 50

    def set_mast_retract(self, target):
        self.events.append(('retract', target))
        self.height = target + 50

    def set_mast_stop(self):
        self.events.append(('mast_stop',))

    def set_ptu_move(self, pan, tilt):
        self.events.append(('ptu', pan, tilt))
        return SimpleNamespace(command_id='test')

    def set_ptu_stop(self):
        self.events.append(('ptu_stop',))

    def get_command_async(self, command_id):
        return SimpleNamespace(result=lambda timeout: SimpleNamespace(
            state=SimpleNamespace(value='completed' if self.completed else 'running')))

    @staticmethod
    def _mast_height(status):
        return status['devices']['mast']['measurements']['height_mm']['value']

    def _require_connected_client(self):
        return self

    def get_status(self, timeout):
        def measurement(value):
            return dict(supported=True, valid=True, fresh=True, value=value)
        return {'devices': {'ptu': {'measurements': {
            'pan_deg': measurement(20), 'tilt_deg': measurement(5)}},
            'mast': {'measurements': {'height_mm': measurement(self.height)},
                     'motion': measurement('STOPPED')}}}


class POIInspectionTest(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.des = FakeDes()
        self.drive = SimpleNamespace(is_paused_for_poi=False, aborted=False)
        self.drive._apply_stop_command = lambda: None
        self.drive.pause_for_poi = lambda: setattr(self.drive, 'is_paused_for_poi', True)
        self.drive.abort_mission = lambda: setattr(self.drive, 'aborted', True)
        base = SimpleNamespace(is_connected=True, last_velocity_feedback_time=100.0,
                               parsed_can_status={'vehicle_velocity_val': 0.0})
        robot = SimpleNamespace(devices={'des_control': self.des}, drive_executor=self.drive,
                                drive_base=base, simulation_mode=False)
        self.manager = MissionManager(robot=robot)
        self.manager.is_active = True
        self.poi = {'mast_height_mm': 4000}

    def execute(self):
        with patch('core.plugin.mission_manager.time.monotonic', self.clock.monotonic), \
             patch('core.plugin.mission_manager.time.sleep', self.clock.sleep):
            return self.manager._execute_real_poi_inspection(self.poi)

    def test_sequence_and_height_tolerance(self):
        self.assertTrue(self.execute())
        self.assertEqual(self.des.events, [('extend', 4000.0), ('mast_stop',),
                                         ('ptu', 35.0, 5.0), ('ptu', 20.0, 5.0),
                                         ('retract', 2900.0), ('mast_stop',)])
        self.assertAlmostEqual(self.clock.now, 102.0)
        self.assertTrue(self.poi['executed'])
        self.assertFalse(self.drive.is_paused_for_poi)

    def test_ptu_incomplete_does_not_retract_or_resume(self):
        self.des.completed = False
        self.assertFalse(self.execute())
        self.assertTrue(self.drive.is_paused_for_poi)
        self.assertTrue(self.drive.aborted)
        self.assertFalse(any(event[0] == 'retract' for event in self.des.events))
        self.assertIn(('ptu_stop',), self.des.events)

    def test_unconfirmed_stop_does_not_move_mast(self):
        self.manager.robot.drive_base.parsed_can_status.clear()
        self.assertFalse(self.execute())
        self.assertNotIn('executed', self.poi)
        self.assertTrue(self.drive.is_paused_for_poi)
        self.assertTrue(self.drive.aborted)
        self.assertFalse(any(event[0] == 'extend' for event in self.des.events))

    def test_retraction_failure_does_not_resume(self):
        def stalled(target):
            self.des.events.append(('retract', target))
        self.des.set_mast_retract = stalled
        self.assertFalse(self.execute())
        self.assertNotIn('executed', self.poi)
        self.assertTrue(self.drive.is_paused_for_poi)
        self.assertTrue(self.drive.aborted)
        self.assertIn(('mast_stop',), self.des.events)


if __name__ == '__main__':
    unittest.main()
