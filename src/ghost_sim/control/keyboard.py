"""World-axis flight keyboard; hold/repeat keys, release to hover."""
import select
import sys
import termios
import tty
import time
import rclpy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('ghost_keyboard')
    publisher = node.create_publisher(Twist,'/ghost/cmd_vel',1)
    cancel = node.create_client(Trigger,'/ghost/cancel_navigation')
    saved = termios.tcgetattr(sys.stdin)
    bindings = {'w':(.3,0,0,0),'s':(-.3,0,0,0),'a':(0,.3,0,0),'d':(0,-.3,0,0),
                'r':(0,0,.25,0),'f':(0,0,-.25,0),'j':(0,0,0,.6),'l':(0,0,0,-.6)}
    print('W/S: world +/-X | A/D: world +/-Y | R/F: up/down | J/L: turn\n'
          'Space/K: hover | Q: quit. Hold/repeat keys; idle 0.35s stops.\n'
          'Movement axes stay fixed to the room even after turning.',flush=True)
    values = (0,0,0,0)
    last = 0.
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            if select.select([sys.stdin],[],[],.05)[0]:
                key = sys.stdin.read(1).lower()
                if key in ('q','\x03',''):
                    break
                if key in (' ','k') and cancel.service_is_ready():
                    cancel.call_async(Trigger.Request())
                values = bindings.get(key,(0,0,0,0))
                last = time.monotonic()
            if time.monotonic()-last > .35:
                values = (0,0,0,0)
            msg = Twist()
            msg.linear.x,msg.linear.y,msg.linear.z,msg.angular.z = map(float,values)
            publisher.publish(msg)
            rclpy.spin_once(node,timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            publisher.publish(Twist()); time.sleep(.02)
        termios.tcsetattr(sys.stdin,termios.TCSADRAIN,saved)
        node.destroy_node(); rclpy.try_shutdown()

if __name__ == '__main__':
    main()
