"""Software interlock prototype. Independent hardware stop/watchdogs required."""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

class SafetySupervisor(Node):
    def __init__(self):
        super().__init__('safety_supervisor')
        for name, value in [('feedback_timeout',0.3),('command_timeout',0.2),('max_linear',0.1),('max_angular',0.2)]:
            self.declare_parameter(name,value)
        self.declare_parameter('require_person_clear',False)
        self.declare_parameter('require_hardware_feedback',True)
        self.declare_parameter('auto_resume_person',True)
        self.input_keys = ()
        if self.get_parameter('require_hardware_feedback').value:
            self.input_keys = ('estop_ok','obstacle_clear','arm_stowed')
        if self.get_parameter('require_person_clear').value:
            self.input_keys += ('person_clear',)
        if not self.input_keys:
            raise ValueError('at least one safety input must be required')
        self.auto_resume_person = ('person_clear' in self.input_keys
                                   and self.get_parameter('auto_resume_person').value)
        self.person_hold = self.auto_resume_person
        self.inputs = {}
        self.command = None
        self.command_time = 0.0
        self.armed = False
        self.pub = self.create_publisher(Twist,'/cmd_vel_safe',10)
        self.status = self.create_publisher(String,'/safety/status',10)
        for key in self.input_keys:
            self.create_subscription(Bool,'/safety/input/'+key,lambda msg,k=key:self.receive(k,msg),10)
        self.create_subscription(Twist,'/cmd_vel_requested',self.receive_command,10)
        self.create_service(Trigger,'/safety/reset',self.reset)
        self.create_timer(0.02,self.tick)

    def receive(self,key,msg):
        self.inputs[key] = (msg.data,time.monotonic())
        self.refresh_person_hold()
        if not msg.data:
            if key != 'person_clear' or not self.auto_resume_person:
                self.armed = False
            self.pub.publish(Twist())

    def refresh_person_hold(self):
        if not self.auto_resume_person:
            return
        value, stamp = self.inputs.get('person_clear',(False,0.0))
        blocked = not value or time.monotonic()-stamp > self.get_parameter('feedback_timeout').value
        # Discard commands during a hold and at release. Only a command received
        # after a fresh clear feedback may move the base; never replay an old one.
        if blocked or self.person_hold != blocked:
            self.command = None
        self.person_hold = blocked

    def receive_command(self,msg):
        values = (msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.x,msg.angular.y,msg.angular.z)
        if not all(math.isfinite(v) for v in values):
            self.armed = False
            self.command = None
            self.pub.publish(Twist())
            return
        self.refresh_person_hold()
        if not self.person_hold:
            self.command,self.command_time = msg,time.monotonic()

    def fault(self,include_person=True):
        now = time.monotonic()
        timeout = self.get_parameter('feedback_timeout').value
        for key in self.input_keys:
            if key == 'person_clear' and not include_person:
                continue
            value,stamp = self.inputs.get(key,(False,0.0))
            if not value or now-stamp > timeout:
                return key+' invalid or stale'
        return ''

    def reset(self,request,response):
        reason = self.fault()
        self.armed = not bool(reason)
        # Require a new command after manual reset.
        self.command = None
        response.success = self.armed
        response.message = reason or 'armed; awaiting fresh command'
        return response

    def tick(self):
        reason = self.fault(include_person=not self.auto_resume_person)
        if reason:
            self.armed = False
        self.refresh_person_hold()
        if not reason and self.person_hold:
            reason = 'person input blocked or stale; awaiting fresh clearance'
        cmd_fresh = self.command is not None and time.monotonic()-self.command_time <= self.get_parameter('command_timeout').value
        if self.armed and not cmd_fresh and self.command is not None:
            self.armed = False
            reason = 'command timeout; reset required'
        out = Twist()
        if self.armed and cmd_fresh and not self.person_hold:
            linear = max(0.0,self.get_parameter('max_linear').value)
            angular = max(0.0,self.get_parameter('max_angular').value)
            out.linear.x = max(-linear,min(linear,self.command.linear.x))
            out.angular.z = max(-angular,min(angular,self.command.angular.z))
        self.pub.publish(out)
        self.status.publish(String(data=reason or ('RUN' if self.armed and cmd_fresh else 'WAIT')))

def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()
