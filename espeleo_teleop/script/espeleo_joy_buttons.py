#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import dynamic_reconfigure.client
import time

LIGHTOFF = 0
LIGHTMEDIUM = 50
LIGHTHIGH = 99


switch_light_back = 0
switch_light_front = 0

def dyn_config_callback(cfg):
    print(cfg)

def callback(data):
    global switch_light_back
    global switch_light_front


    #Espeleo Turbo Control button
    if (data.buttons[1] == 1):
	client.update_configuration({"turbo_button":True})
    else:
	client.update_configuration({"turbo_button":False})


    # Espeleo Orientation button
    if (data.buttons[3] == 1) : 
        getforwardinfo = rospy.get_param('/espeleo/forward_orientation')
        if getforwardinfo == True:
            client.update_configuration({"forward_orientation":False})

        if getforwardinfo == False:
            client.update_configuration({"forward_orientation":True})

    # Espeleo Backlight button
    if (data.buttons[4] == 1):
	print("backlight", switch_light_back)
        switch_light_back += 1

        if switch_light_back == 1:
            client.update_configuration({"backlight_slider":LIGHTOFF})
            client.update_configuration({"backlight_button": False})
        elif switch_light_back == 2:
            client.update_configuration({"backlight_slider":LIGHTMEDIUM})
            client.update_configuration({"backlight_button": True})
        elif switch_light_back == 3:
            client.update_configuration({"backlight_slider":LIGHTHIGH})
            client.update_configuration({"backlight_button": True})
        
        if switch_light_back >= 3:
            switch_light_back = 0
        
    # Espeleo Frontlight button
    if (data.buttons[5] == 1):

        switch_light_front += 1

        if switch_light_front == 1:
            client.update_configuration({"frontlight_slider":LIGHTOFF})
            client.update_configuration({"frontlight_button": False})
        elif switch_light_front == 2:
            client.update_configuration({"frontlight_slider":LIGHTMEDIUM})
            client.update_configuration({"frontlight_button": True})
        elif switch_light_front == 3:
            client.update_configuration({"frontlight_slider":LIGHTHIGH})
            client.update_configuration({"frontlight_button": True})

        if switch_light_front >= 3:
            switch_light_front = 0

    rospy.loginfo("Config set to: {}".format(data))

if __name__ == '__main__':
    rospy.init_node('espeleo_joystick', anonymous=True)

    rospy.Subscriber("/joy", Joy, callback)
    client = dynamic_reconfigure.client.Client("espeleo", timeout=5, config_callback=dyn_config_callback)
    r = rospy.Rate(5)
    #b = False
    while not rospy.is_shutdown():
        #b = not b

        #client.update_configuration({"forward_orientation":True})
        #time.sleep(2)
        #client.update_configuration({"forward_orientation":False})
        #time.sleep(2)
        r.sleep()
