#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
import easygopigo3 as easy

gpg = easy.EasyGoPiGo3()


def callback(data):
    t = data.linear.x;
    r = data.angular.z;

    mag = math.sqrt(pow(t,2)+pow(r,2));
    if mag <= 0.1:
        motor1(0);
        motor2(0);
    else:
        m1 = 0;
        m2 = 0;

        m1 = t/mag * abs(t);
        m2 = t/mag * abs(t);

        m1 += r/mag * abs(r);
        m2 += -r/mag * abs(r);

        m1 = m1/abs(m1) * min(abs(m1), 1.0);
        m2 = m2/abs(m2) * min(abs(m2), 1.0);

        motor1(m1*255)
        motor2(m2*255)



def motor1(dps):
    gpg.set_motor_dps(gpg.MOTOR_RIGHT, dps)


def motor2(dps):
    gpg.set_motor_dps(gpg.MOTOR_LEFT, dps)



def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)

    lwheel = rospy.Publisher("lwheel", Int16, queue_size=1)
    rwheel = rospy.Publisher("rwheel", Int16, queue_size=1)

    init_encoders = gpg.read_encoders()
    init_lwheel   = init_encoders[0]
    init_rwheel   = init_encoders[1]

    r = rospy.Rate(20)

    while not rospy.is_shutdown():
        encoders = gpg.read_encoders()
        lwheel.publish(encoders[0] - init_encoders[0])
        rwheel.publish(encoders[1] - init_encoders[1])
        r.sleep()
    motor1(0)
    motor2(0)


if __name__ == '__main__':
    listener()


# in one console : roscore
# in another console : python gopigo3_ros.py
# in another console : 
# cmd 1 : rostopic pub cmd_vel geometry_msgs/Twist '[0.2, 0.0, 0.0]' '[0.0, 0.0, 0.0]'
# cmd 2 : rostopic pub cmd_vel geometry_msgs/Twist '[0.0, 0.0, 0.0]' '[0.0, 0.0, 0.2]'
# based one explanation from : https://titanwolf.org/Network/Articles/Article?AID=ffc27712-a7fc-4ccf-84b8-0cc8d235a94f#gsc.tab=0