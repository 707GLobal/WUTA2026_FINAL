import importlib.util
from pathlib import Path
import tempfile
import unittest


SCRIPT = (Path(__file__).resolve().parents[1] / 'humble-chcnav-cgi_ros2pkg'
          / 'src/chcnav/scripts/check_serial_port.py')
spec = importlib.util.spec_from_file_location('serial_guard', SCRIPT)
guard = importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


class SerialGuardTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.proc = self.root / 'proc'
        self.proc.mkdir()

    def reader(self, pid, device):
        process = self.proc / str(pid)
        (process / 'fd').mkdir(parents=True)
        (process / 'comm').write_text('HcMsgParserLaun\n')
        (process / 'fd/3').symlink_to(device)
        (process / 'fd/4').symlink_to(device)

    def test_free_device(self):
        self.reader(20, '/dev/zero')
        self.assertEqual(guard.port_users('/dev/null', self.proc), [])

    def test_alias_and_multiple_descriptors(self):
        alias = self.root / 'ins'
        alias.symlink_to('/dev/null')
        self.reader(20, '/dev/null')
        self.reader(10, alias)
        self.assertEqual(guard.port_users(alias, self.proc),
                         [(10, 'HcMsgParserLaun'), (20, 'HcMsgParserLaun')])

    def test_exited_process_and_closed_descriptor(self):
        (self.proc / '10').mkdir()
        self.reader(20, self.root / 'missing')
        self.assertEqual(guard.port_users('/dev/null', self.proc), [])

    def test_reject_regular_file(self):
        regular = self.root / 'file'
        regular.touch()
        with self.assertRaises(ValueError):
            guard.port_users(regular, self.proc)


if __name__ == '__main__':
    unittest.main()
