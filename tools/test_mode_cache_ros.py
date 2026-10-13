"""Run the real ROS node with a fake CAN socket; never accesses a CAN bus."""
import os
import pathlib
import shlex
import signal
import subprocess
import unittest
from tools.test_mode_cache import ModeCacheTests, ROOT, PACKAGE
SOCKET = r'''
#include "can_interface/can_socket.hpp"
#include <cstdlib>
#include <cassert>
namespace can_interface {
CanSocket::~CanSocket() {}
bool CanSocket::open(const std::string &) { return true; }
void CanSocket::close() {}
bool CanSocket::send(const CanFrame &) { assert(false); return false; }
bool CanSocket::receive(CanFrame &frame) {
  static int count=0;
  if (std::getenv("CACHE_TEST_LIVE") && ++count==1) {
    frame.can_id=0x501; frame.dlc=8; frame.data[0]=4; return true;
  }
  return false;
}
}
'''
class ModeCacheRosTests(ModeCacheTests):
    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        folder=pathlib.Path(cls.build.name)
        source=folder/'socket.cpp'; source.write_text(SOCKET)
        obj=folder/'socket.o'
        subprocess.run(['c++','-std=c++17','-I',str(PACKAGE/'include'),'-c',str(source),'-o',str(obj)],check=True)
        build=ROOT/'WUTA-FSD/ros2_ws/build/can_interface'
        command=shlex.split((build/'CMakeFiles/can_interface_node.dir/link.txt').read_text())
        cls.node=folder/'can_interface_node'
        command[command.index('-o')+1]=str(cls.node)
        command[command.index('libcan_socket.a')]=str(obj)
        subprocess.run(command,cwd=build,check=True)
    def run_node(self,live=False):
        env=dict(os.environ,ROS_DOMAIN_ID='173',ROS_LOCALHOST_ONLY='1')
        if live: env['CACHE_TEST_LIVE']='1'
        command=[str(self.node),'--ros-args','-p','frames.control.period_sec:=0.0',
                 '-p','frames.heartbeat.period_sec:=0.0','-p',f'mode_cache.directory:={self.directory}',
                 '-p','mode_cache.bootstrap_wait_sec:=0.1']
        process=subprocess.Popen(command,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
        try:
            output,_=process.communicate(timeout=1.5)
            self.fail(f'node unexpectedly exited: {output}')
        except subprocess.TimeoutExpired:
            process.send_signal(signal.SIGINT)
            output,_=process.communicate(timeout=5)
        self.assertEqual(process.returncode,0,output)
        return output
    def test_node_recovers_mode_for_late_start(self):
        self.update(2); output=self.run_node()
        self.assertIn('Restored VCU mode 2 from bridge cache',output)
        self.assertIn("mission mode 'acceleration'",output)
        self.assertNotIn('start command',output)
    def test_node_live_mode_wins_over_cache(self):
        self.update(2); output=self.run_node(live=True)
        self.assertIn("mission mode 'skidpad'",output)
        self.assertNotIn('Restored VCU mode',output)
        self.assertNotIn("mission mode 'acceleration'",output)
    def test_node_empty_cache_keeps_waiting_for_live_mode(self):
        output=self.run_node()
        self.assertIn('Mode cache bootstrap skipped:',output)
        self.assertNotIn("mission mode '",output)
if __name__=='__main__':
    unittest.main()
