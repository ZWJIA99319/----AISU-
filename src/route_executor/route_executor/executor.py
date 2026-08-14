"""Route executor using immutable snapshots and transition functions."""
import math
from dataclasses import dataclass, replace
from time import monotonic

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

START = (35., 20.)
COMMON_IN = (START, (420.,130.), (240.,170.), (240.,280.))
CCW = ((300.,320.),(330.,320.),(410.,375.),(330.,440.),(250.,440.),
       (100.,435.),(70.,375.),(100.,320.),(150.,320.))
CW = tuple(reversed(CCW))
COMMON_OUT = ((220.,260.),(220.,180.),(50.,20.))


@dataclass(frozen=True)
class Snapshot:
    target: int = 1
    mode: str = 'drive'
    smoothed: float = 0.0
    reverse_at: float = 0.0


def relative_route(selection):
    points = COMMON_IN + (CCW if selection == 1 else CW) + COMMON_OUT
    return tuple((x-START[0], y-START[1]) for x,y in points)


def normalize(angle):
    return math.degrees(math.atan2(math.sin(math.radians(angle)), math.cos(math.radians(angle))))


def transition(state, route, pose, clock, velocity, proportional_gain):
    """Compute (next snapshot, linear speed, angular speed) without ROS side effects."""
    if state.mode == 'done': return state, 0., 0.
    if state.mode == 'reverse':
        if clock - state.reverse_at < 1.35: return state, -1., 5.
        return replace(state, target=state.target+1, mode='drive', smoothed=0.), 0., 0.
    if state.target >= len(route): return replace(state, mode='done'), 0., 0.
    x,y,yaw = pose; gx,gy = route[state.target]
    dx,dy = gx-x,gy-y
    if math.hypot(dx,dy) <= 40.:
        if state.target == 1:
            return replace(state, mode='reverse', reverse_at=clock, smoothed=0.), 0., 0.
        if state.target == len(route)-1:
            return replace(state, mode='done', smoothed=0.), 0., 0.
        return replace(state, target=state.target+1, smoothed=0.), 0., 0.
    raw = normalize(yaw-math.degrees(math.atan2(dy,dx)))
    smooth = 0. if abs(raw) <= 3. else .7*raw+.3*state.smoothed
    return replace(state, smoothed=smooth), velocity, max(-5.,min(5.,smooth*proportional_gain))


class Executor(Node):
    def __init__(self):
        super().__init__('route_executor')
        for key,value in [('line_v',1.0),('line_kp',-0.030),('route_mode',1)]:
            self.declare_parameter(key,value)
        self.route = relative_route(int(self.get_parameter('route_mode').value))
        self.velocity = float(self.get_parameter('line_v').value)
        self.gain = float(self.get_parameter('line_kp').value)
        self.state, self.pose = Snapshot(), None
        qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
        self.output=self.create_publisher(Twist,'/cmd_vel',qos)
        self.create_subscription(Odometry,'/odom_combined',self.receive,qos)
        self.create_timer(.01,self.update)

    def receive(self,msg):
        p,q=msg.pose.pose.position,msg.pose.pose.orientation
        yaw=math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
        self.pose=(p.x*100.,p.y*100.,yaw)

    def update(self):
        if self.pose is None:return
        self.state,linear,angular=transition(self.state,self.route,self.pose,monotonic(),self.velocity,self.gain)
        msg=Twist();msg.linear.x=linear;msg.angular.z=angular;self.output.publish(msg)

    def stop(self):
        for _ in range(5):self.output.publish(Twist())


def main(args=None):
    rclpy.init(args=args);node=Executor()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.stop();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
