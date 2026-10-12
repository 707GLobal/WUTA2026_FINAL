"""WUTA-SIM 本地适配：用开度计算加速度，保留原模型横向动力学。"""
import math,time
import rclpy
from vehicle_model.vehicle_model import VehicleModel
class EffortVehicle(VehicleModel):
    def __init__(self):
        self.effort=0.;self.last_command=0.
        super().__init__()
    def on_command(self,msg):
        if not all(math.isfinite(v) for v in (msg.angle,msg.speed,msg.throttle_brake)):
            self.effort=0.;self.cmd_angle=0.;return
        self.effort=max(-1.,min(1.,msg.throttle_brake))
        self.cmd_angle=msg.angle;self.last_command=time.monotonic()
    def step(self):
        u=self.effort if time.monotonic()-self.last_command<.3 else 0.
        a=u*(self.max_accel if u>=0 else self.max_decel)-.05*self.v
        a=max(-max(0.,self.v)/self.dt,a)
        self.cmd_speed=self.v+a/max(self.speed_gain,1e-6)
        super().step()
        self.v=max(0.,self.v)
def main():
    rclpy.init();n=EffortVehicle()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
