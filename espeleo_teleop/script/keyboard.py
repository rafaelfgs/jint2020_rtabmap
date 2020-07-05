#!/usr/bin/env python

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

import roslib
import rospy

from geometry_msgs.msg import Twist

import sys, select, termios, tty

msg = """
---------------------------------------------------
Reading from the keyboard and Publishing to Twist!

Moving arrows: <  ^  >

t : up (+z)
b : down (-z)

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%

CTRL-C to quit
---------------------------------------------------
"""

moveBindings = {
        '\x1b[A':(1,0,0,0),
        '\x1b[D':(0,0,0,1),
        '\x1b[C':(0,0,0,-1),
        '\x1b[B':(-1,0,0,0),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

def setup_term(fd, when=termios.TCSAFLUSH):
    mode = termios.tcgetattr(fd)
    mode[tty.LFLAG] = mode[tty.LFLAG] & ~(termios.ECHO | termios.ICANON)
    termios.tcsetattr(fd, when, mode)

def getch(timeout=None):
	settings = termios.tcgetattr(sys.stdin)
	try:
		setup_term(sys.stdin)
		try:
			rw, wl, xl = select.select([sys.stdin], [], [], timeout)
		except select.error:
			return
		if rw:
			key = sys.stdin.read(1)

			if key == '\x1b':
				key += sys.stdin.read(2)

			return key
	finally:
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)
    rospy.init_node('teleop_node')

    speed = rospy.get_param("~speed", 1.0)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        while not rospy.is_shutdown():
            key = getch(0.5)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print(vels(speed,turn))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)