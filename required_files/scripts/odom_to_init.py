#!/usr/bin/python
import rospy
import sys
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage



tf_init  = (376, 80, -5.75, 0, 0, 0.8191520, 0.5735764)
tf_d435i = (0.07, 0, 0.25, -0.5, 0.5, -0.5, 0.5)



def qq_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return x, y, z, w

def q_conjugate(q):
    x, y, z, w = q
    return -x, -y, -z, w

def v_conjugate(v):
    x, y, z, = v
    return -x, -y, -z

def qv_mult(q1, v1):
    q2 = v1 + (0.0,)
    return qq_mult(qq_mult(q1, q2), q_conjugate(q1))[:-1]



def callback_rtab(data):
    
    odom = Odometry()
    
    odom.header.seq = data.header.seq
    odom.header.stamp = data.header.stamp
    odom.header.frame_id = "base_init"
    odom.child_frame_id = "rtab_link"
    
    v_sensor = (data.pose.pose.position.x, data.pose.pose.position.y, data.pose.pose.position.z)
    q_sensor = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    
    v_base = qv_mult(q_sensor, qv_mult(q_conjugate(tf_d435i[3:]), v_conjugate(tf_d435i[:3])))
    v_base = qv_mult(tf_d435i[3:], (v_sensor[0]+v_base[0], v_sensor[1]+v_base[1], v_sensor[2]+v_base[2]))
    v_base = (v_base[0]+tf_d435i[0], v_base[1]+tf_d435i[1], v_base[2]+tf_d435i[2])
    q_base = qq_mult(tf_d435i[3:], qq_mult(q_sensor, q_conjugate(tf_d435i[3:])))
    
    odom.pose.pose.position.x = v_base[0]
    odom.pose.pose.position.y = v_base[1]
    odom.pose.pose.position.z = v_base[2]
    
    odom.pose.pose.orientation.x = q_base[0]
    odom.pose.pose.orientation.y = q_base[1]
    odom.pose.pose.orientation.z = q_base[2]
    odom.pose.pose.orientation.w = q_base[3]
    
    pub = rospy.Publisher("/odom", Odometry, queue_size=10)
    pub.publish(odom)



def callback_truth(data):
    
    if data.transforms[0].header.frame_id == "world" and  data.transforms[0].child_frame_id == "base_link":
        
        odom = Odometry()
        
        odom.header.seq = data.transforms[0].header.seq
        odom.header.stamp = data.transforms[0].header.stamp
        odom.header.frame_id = "base_init"
        odom.child_frame_id = "truth_link"
        
        v_world = (data.transforms[0].transform.translation.x, data.transforms[0].transform.translation.y, data.transforms[0].transform.translation.z)
        q_world = (data.transforms[0].transform.rotation.x, data.transforms[0].transform.rotation.y, data.transforms[0].transform.rotation.z, data.transforms[0].transform.rotation.w)
        
        #v_init = qv_mult(q_world, tf_init[:3])
        v_init = qv_mult(q_conjugate(tf_init[3:]), (v_world[0]-tf_init[0], v_world[1]-tf_init[1], v_world[2]-tf_init[2]))
        q_init = qq_mult(q_conjugate(tf_init[3:]), q_world)
        
        odom.pose.pose.position.x = v_init[0]
        odom.pose.pose.position.y = v_init[1]
        odom.pose.pose.position.z = v_init[2]
        
        odom.pose.pose.orientation.x = q_init[0]
        odom.pose.pose.orientation.y = q_init[1]
        odom.pose.pose.orientation.z = q_init[2]
        odom.pose.pose.orientation.w = q_init[3]
        
        pub = rospy.Publisher("/truth", Odometry, queue_size=10)
        pub.publish(odom)



def main_function():
    
    global f_rtab, f_truth
    
    rospy.init_node("odom_to_init", anonymous=True)
    
    rospy.Subscriber("/rtabmap/odom", Odometry, callback_rtab)
    rospy.Subscriber("/tf", TFMessage, callback_truth)
    
    rate = rospy.Rate(10)
    
    sys.stdout.write("Saving Odometry to File...\n")
    sys.stdout.flush()
    
    while not rospy.is_shutdown():
        rate.sleep()



if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass