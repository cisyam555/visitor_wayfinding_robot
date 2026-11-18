"""Independent backend checks; starts no ROS node or robot service."""
import ast
import json
import threading
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from unittest.mock import patch

import monitor


class MonitorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.state = monitor.State()
        cls.state.places = [{'name': '跆拳道馆', 'poi': '跆拳道馆'}]
        cls.server = monitor.Server(('127.0.0.1', 0), monitor.Handler)
        cls.server.state = cls.state
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.base = f'http://127.0.0.1:{cls.server.server_port}'

    @classmethod
    def tearDownClass(cls):
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(2)

    def post(self, path, payload, method='POST'):
        request = urllib.request.Request(
            self.base+path, data=json.dumps(payload).encode(),
            headers={'Content-Type': 'application/json'}, method=method)
        return urllib.request.urlopen(request, timeout=15)

    def test_status_reports_task_places_and_node(self):
        value = json.load(urllib.request.urlopen(self.base+'/api/status'))
        self.assertIs(value['read_only'], True)
        self.assertEqual(value['task']['state'], 'idle')
        self.assertEqual(value['places'][0]['name'], '跆拳道馆')
        self.assertIn('running', value['node'])

    def test_control_requires_confirmation(self):
        with self.assertRaises(urllib.error.HTTPError) as result:
            self.post('/api/navigate', {'place': '跆拳道馆'})
        self.assertEqual(result.exception.code, 400)

    def test_navigate_rejects_unknown_place(self):
        with self.assertRaises(urllib.error.HTTPError) as result:
            self.post('/api/navigate', {'place': '不存在', 'confirm': True})
        self.assertEqual(result.exception.code, 400)

    def test_navigate_requires_running_voice_node(self):
        with patch.object(self.state, 'voice_running', return_value=False):
            with self.assertRaises(urllib.error.HTTPError) as result:
                self.post('/api/navigate', {'place': '跆拳道馆', 'confirm': True})
        self.assertEqual(result.exception.code, 409)

    def test_unknown_post_path_is_not_found(self):
        with self.assertRaises(urllib.error.HTTPError) as result:
            self.post('/api/start', {'confirm': True})
        self.assertEqual(result.exception.code, 404)

    def test_other_write_methods_are_rejected(self):
        for method in ['PUT', 'PATCH', 'DELETE']:
            with self.subTest(method=method):
                with self.assertRaises(urllib.error.HTTPError) as result:
                    self.post('/api/node/start', {'confirm': True}, method=method)
                self.assertEqual(result.exception.code, 405)

    def test_no_directory_or_source_access(self):
        for path in ['/monitor.py', '/start_monitor.sh', '/test_monitor.py', '/../README.md']:
            with self.subTest(path=path):
                with self.assertRaises(urllib.error.HTTPError) as result:
                    urllib.request.urlopen(self.base+path)
                self.assertEqual(result.exception.code, 404)

    def test_stale_camera_is_not_served(self):
        with self.state.lock:
            self.state.frames['color'] = {'at': time.time()-10, 'jpeg': b'old', 'width': 1, 'height': 1}
        with self.assertRaises(urllib.error.HTTPError) as result:
            urllib.request.urlopen(self.base+'/api/frame?kind=color')
        self.assertEqual(result.exception.code, 503)

    def test_snapshot_excludes_image_bytes_and_marks_age(self):
        self.state.sample('test', {'value': 5})
        with self.state.lock:
            self.state.frames['color'] = {'at': time.time(), 'jpeg': b'bytes', 'width': 1, 'height': 1}
        value = self.state.snapshot()
        self.assertGreaterEqual(value['samples']['test']['age_s'], 0)
        self.assertNotIn('jpeg', value['cameras']['color'])
        json.dumps(value, allow_nan=False)

    def test_nonfinite_and_credential_redaction(self):
        self.assertIsNone(monitor.finite(float('nan')))
        self.assertIsNone(monitor.finite(float('inf')))
        redacted = monitor.redact('api_key=private_value access_token: private_token Bearer private_bearer')
        for secret in ['private_value', 'private_token', 'private_bearer']:
            self.assertNotIn(secret, redacted)

    def test_chassis_battery_uses_explicit_charging_flag(self):
        parsed = monitor.parse_chassis_battery({'success': True, 'batteryPercentage': 62, 'current': -31003, 'isCharging': True})
        self.assertEqual(parsed['percent'], 62)
        self.assertIs(parsed['is_charging'], True)
        self.assertIsNone(monitor.parse_chassis_battery({'success': True, 'batteryPercentage': 0})['is_charging'])
        for data in [{'success': False, 'batteryPercentage': 62}, {'success': True, 'batteryPercentage': -1}, {'success': True, 'batteryPercentage': float('nan')}, {'success': True, 'batteryPercentage': True}]:
            with self.subTest(data=data), self.assertRaises(ValueError):
                monitor.parse_chassis_battery(data)

    def test_task_tracking_follows_navigation_logs(self):
        state = monitor.State()
        state.track_task('开始定点导航：跆拳道馆（跆拳道馆，送访客）')
        self.assertEqual(state.task['state'], 'running')
        state.track_task('定点导航到 跆拳道馆：3 个导航段，乐知教育(9.28,1.97) -> 厕所(27.68,2.74) -> 跆拳道馆(31.45,2.51)')
        self.assertEqual(state.task['route'], ['乐知教育', '厕所', '跆拳道馆'])
        state.track_task('到达 厕所，用时 16.4s')
        self.assertEqual(state.task['state'], 'running')
        state.track_task('到达 跆拳道馆，用时 10.0s')
        self.assertEqual(state.task['state'], 'done')

    def test_backend_never_calls_chassis_sdk_or_shell(self):
        tree = ast.parse(Path(monitor.__file__).read_text())
        for node in ast.walk(tree):
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
                self.assertNotIn(node.func.attr, {'call_async', 'send_goal_async', 'system'})
            if isinstance(node, ast.keyword) and node.arg == 'shell':
                self.assertIs(node.value.value, False)


if __name__ == '__main__':
    unittest.main()
