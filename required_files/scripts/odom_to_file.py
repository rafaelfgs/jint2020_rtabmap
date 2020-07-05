#!/usr/bin/python
import rospy
import sys
import os
from copy import copy
from nav_msgs.msg import Odometry, Path



p_odom  = "/home/rafael/Dropbox/UFMG/Artigos/jint2020/odom/cave/5/odom.txt"
p_path  = "/home/rafael/Dropbox/UFMG/Artigos/jint2020/odom/cave/5/path.txt"
p_truth = "/home/rafael/Dropbox/UFMG/Artigos/jint2020/odom/cave/5/truth.txt"

t_init = 0.0
n_odom = 0
n_path = 0
t_path = []
n_truth = 0

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



def callback_odom(data):
    
    global t_init, n_odom
    
    if t_init == 0.0: t_init = data.header.stamp.to_sec()
    n_odom += 1
    
    f_odom.write("\n%d %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f" % (n_odom,
                                                                           data.header.stamp.to_sec() - t_init,
                                                                           copy(data.pose.pose.position.x),
                                                                           copy(data.pose.pose.position.y),
                                                                           copy(data.pose.pose.position.z),
                                                                           copy(data.pose.pose.orientation.x),
                                                                           copy(data.pose.pose.orientation.y),
                                                                           copy(data.pose.pose.orientation.z),
                                                                           copy(data.pose.pose.orientation.w)))


def callback_path(data):
    
    global t_init, n_path, t_path, x_path
    
    if t_init == 0.0: t_init = data.header.stamp.to_sec()
    
    if len(data.poses) > n_path:
        n_path = len(data.poses)
        t_path += [data.header.stamp.to_sec() - t_init]
        x_path = data.poses



def callback_truth(data):
    
    global t_init, n_truth
    
    if t_init == 0.0: t_init = data.header.stamp.to_sec()
    n_truth += 1
    
    f_truth.write("\n%d %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f" % (n_truth,
                                                                            data.header.stamp.to_sec() - t_init,
                                                                            copy(data.pose.pose.position.x),
                                                                            copy(data.pose.pose.position.y),
                                                                            copy(data.pose.pose.position.z),
                                                                            copy(data.pose.pose.orientation.x),
                                                                            copy(data.pose.pose.orientation.y),
                                                                            copy(data.pose.pose.orientation.z),
                                                                            copy(data.pose.pose.orientation.w)))



def save_path():
    
    for k in range(len(x_path)):
        
        v_sensor = (x_path[k].pose.position.x, x_path[k].pose.position.y, x_path[k].pose.position.z)
        q_sensor = (x_path[k].pose.orientation.x, x_path[k].pose.orientation.y,
                    x_path[k].pose.orientation.z, x_path[k].pose.orientation.w)
        
        v_base = qv_mult(q_sensor, qv_mult(q_conjugate(tf_d435i[3:]), v_conjugate(tf_d435i[:3])))
        v_base = qv_mult(tf_d435i[3:], (v_sensor[0]+v_base[0], v_sensor[1]+v_base[1], v_sensor[2]+v_base[2]))
        v_base = (v_base[0]+tf_d435i[0], v_base[1]+tf_d435i[1], v_base[2]+tf_d435i[2])
        q_base = qq_mult(tf_d435i[3:], qq_mult(q_sensor, q_conjugate(tf_d435i[3:])))
        
        
        f_path.write("\n%d %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f %+.3f" % (k+1, t_path[k], copy(v_base[0]),
                                                                               copy(v_base[1]), copy(v_base[2]),
                                                                               copy(q_base[0]), copy(q_base[1]),
                                                                               copy(q_base[2]), copy(q_base[3])))



def main_function():
    
    global f_odom, f_path, f_truth
    
    rospy.init_node("odom_to_file", anonymous=True)
    
    if os.path.exists(p_odom):  os.remove(p_odom)
    if os.path.exists(p_path):  os.remove(p_path)
    if os.path.exists(p_truth): os.remove(p_truth)
    
    f_odom = open(p_odom,"w")
    f_odom.write("n t px py pz qx qy qz qw")
    
    f_path = open(p_path,"w")
    f_path.write("n t px py pz qx qy qz qw")
    
    f_truth = open(p_truth,"w")
    f_truth.write("n t px py pz qx qy qz qw")
    
    rospy.Subscriber("/odom", Odometry, callback_odom)
    rospy.Subscriber("/rtabmap/mapPath", Path, callback_path)
    rospy.Subscriber("/truth", Odometry, callback_truth)
    
    rate = rospy.Rate(10)

    sys.stdout.write("Saving Odometry to File...\n")
    sys.stdout.flush()
    
    while not rospy.is_shutdown():
        rate.sleep()
    
    save_path()
    
    f_odom.close()
    f_path.close()
    f_truth.close()



if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass