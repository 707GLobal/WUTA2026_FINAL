"""Exercise real C writer and C++ reader without USB access or CAN transmission."""
import pathlib
import subprocess
import tempfile
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'WUTA-FSD/ros2_ws/src/system/can_interface'
WRITER = r'''
#include "mode_cache.h"
#include <stdio.h>
int main(int argc, char **argv) {
  if (argc!=2 || mode_cache_open(argv[1],"can0")<0) return 2;
  puts("READY"); fflush(stdout);
  unsigned id,mode,dlc;
  while (scanf("%u %u %u",&id,&mode,&dlc)==3) {
    bridge_frame_t f={0}; f.id=id; f.dlc=dlc; f.data[0]=mode;
    mode_cache_update(&f); puts("UPDATED"); fflush(stdout);
  }
  mode_cache_close(); return 0;
}
'''
READER = r'''
#include "can_interface/mode_cache.hpp"
#include <iostream>
int main(int argc,char **argv) {
  if(argc!=4) return 2;
  can_interface::ModeSnapshot s; std::string reason;
  if (!can_interface::readModeCache(argv[1],argv[2],0x501,std::stod(argv[3]),s,reason)) {
    std::cout<<reason; return 1;
  }
  std::cout<<unsigned(s.mode)<<" "<<s.sequence; return 0;
}
'''

class ModeCacheTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.build = tempfile.TemporaryDirectory()
        folder = pathlib.Path(cls.build.name)
        (folder/'writer.c').write_text(WRITER)
        (folder/'reader.cpp').write_text(READER)
        cls.writer = folder/'zlg_can_bridge'
        cls.reader = folder/'reader'
        subprocess.run(['cc','-Wall','-Wextra','-Werror','-I',str(ROOT/'zlgcan_bridge'),
                        str(folder/'writer.c'),str(ROOT/'zlgcan_bridge/mode_cache.c'),
                        str(ROOT/'zlgcan_bridge/logger.c'),'-lpthread','-o',str(cls.writer)],check=True)
        subprocess.run(['c++','-std=c++17','-Wall','-Wextra','-Werror','-I',str(PACKAGE/'include'),
                        str(folder/'reader.cpp'),str(PACKAGE/'src/mode_cache.cpp'),
                        '-o',str(cls.reader)],check=True)

    @classmethod
    def tearDownClass(cls):
        cls.build.cleanup()

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.directory = pathlib.Path(self.tmp.name)
        self.path = self.directory/'can0.mode'
        self.process = self.start_writer()

    def start_writer(self):
        process = subprocess.Popen([str(self.writer),str(self.directory)],stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE,text=True)
        while process.stdout.readline().strip() != 'READY':
            if process.poll() is not None:
                self.fail('writer failed')
        return process

    def tearDown(self):
        if self.process.poll() is None:
            self.process.stdin.close()
            self.process.wait(timeout=5)
        if not self.process.stdin.closed:
            self.process.stdin.close()
        self.process.stdout.close()
        self.tmp.cleanup()

    def update(self,mode=2,id=0x501,dlc=8):
        self.process.stdin.write(f'{id} {mode} {dlc}\n'); self.process.stdin.flush()
        while self.process.stdout.readline().strip() != 'UPDATED':
            if self.process.poll() is not None:
                self.fail('writer failed during update')

    def read(self,iface='can0',age=0):
        return subprocess.run([str(self.reader),str(self.directory),iface,str(age)],
                              capture_output=True,text=True,timeout=5)

    def mutate(self,key,value):
        lines=self.path.read_text().splitlines()
        self.path.write_text('\n'.join(f'{key}={value}' if x.startswith(key+'=') else x for x in lines)+'\n')

    def test_late_reader_and_all_mode_values(self):
        self.assertNotEqual(self.read().returncode,0)
        for sequence,mode in enumerate((2,4,1,0,255,6),1):
            self.update(mode)
            result=self.read()
            self.assertEqual(result.returncode,0,result.stdout)
            self.assertEqual(result.stdout,f'{mode} {sequence}')

    def test_remote_extended_empty_and_res_frames_are_not_cached(self):
        self.update()
        for id,dlc in ((0x501|0x80000000,8),(0x501|0x40000000,8),
                       (0x501|0x20000000,8),(0x501,0),(0x1e4,8)):
            self.update(0x13,id,dlc)
            self.assertEqual(self.read().stdout,'2 1')

    def test_process_death_rejects_leftover_snapshot(self):
        self.update(); self.process.kill(); self.process.wait(timeout=5)
        self.assertTrue(self.path.exists())
        self.assertNotEqual(self.read().returncode,0)

    def test_new_session_and_normal_close(self):
        self.update(); old=self.path.read_text()
        self.process.stdin.close(); self.process.wait(timeout=5); self.process.stdout.close()
        self.assertFalse(self.path.exists())
        self.process=self.start_writer()
        self.assertNotEqual(self.read().returncode,0)
        self.path.write_text(old)
        self.assertNotEqual(self.read().returncode,0)

    def test_invalid_identity_age_and_format(self):
        self.update(); original=self.path.read_text()
        for key,value in (('boot_id','old'),('iface','canB'),('can_id','484'),
                          ('process_start_ticks','1'),('valid','0'),('mode','256'),
                          ('received_monotonic_ns','18446744073709551615'),('sequence','0')):
            with self.subTest(key=key):
                self.path.write_text(original); self.mutate(key,value)
                self.assertNotEqual(self.read().returncode,0)
        self.path.write_text(original); self.mutate('received_monotonic_ns','1')
        self.assertNotEqual(self.read(age=1).returncode,0)
        self.path.write_text('version=1\nmode=2\n')
        self.assertNotEqual(self.read().returncode,0)

    def test_second_writer_cannot_replace_live_cache(self):
        self.update(); original=self.path.read_text()
        result=subprocess.run([str(self.writer),str(self.directory)],capture_output=True,timeout=5)
        self.assertNotEqual(result.returncode,0)
        self.assertEqual(self.path.read_text(),original)

    def test_write_failure_invalidates_old_cache_and_can_recover(self):
        self.update()
        temp=self.directory/f'can0.mode.tmp.{self.process.pid}'
        temp.mkdir()
        self.update(4)
        self.assertFalse(self.path.exists())
        temp.rmdir(); self.update(4)
        self.assertEqual(self.read().stdout,'4 3')

if __name__=='__main__':
    unittest.main()
