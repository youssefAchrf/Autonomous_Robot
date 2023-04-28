#!/usr/bin/env pyhton3
import rospy
import math
import time
from std_msgs.msg import Float32, Float32MultiArray, Int32
def pose_clbk(msg):  # hena bs bysyf el  x wl y wl theta mn el localizeer
    global x, y, theta
    x = msg.data[0]
    y = msg.data[1]
    theta = msg.data[2]

if __name__ == "__main__":
    rightV = 0
    stopsign = 0
    leftV = 0
    speed = 3
    rospy.init_node("navigator_node")
    # Subscriber to the velocity commanding topic
    rospy.Subscriber("/wheel_odometry_localizer/pose", Float32MultiArray, pose_clbk)

    # Setup wheel speed publishers
    LWS_pub = rospy.Publisher("/sim_ros_interface/left_motor/setpoint_speed", Float32, queue_size=10)
    RWS_pub = rospy.Publisher("/sim_ros_interface/right_motor/setpoint_speed", Float32, queue_size=10)

    # Setup ROS rate
    rate = rospy.Rate(20)  # 10 Hz
    rospy.loginfo("Line follower node worked successfully")

    while not rospy.is_shutdown():
        rightV = -1
        leftV = -1
        flagstart = 0
        left_siganl = rospy.wait_for_message("/sim_ros_interface/leftvision/state", Float32)

        right_siganl = rospy.wait_for_message("/sim_ros_interface/rightvision/state", Float32)

        mid_siganl = rospy.wait_for_message("/sim_ros_interface/middlevision/state", Float32)

        proximity_msg = rospy.wait_for_message("/sim_ros_interface/proximity_sensor/state",Int32)  # byceck 3la el sensor

        if (proximity_msg.data) != 0:  # proximity sees something
            rospy.loginfo("i will stop -- obstacle ")
            rightV = 0
            leftV = 0
        elif left_siganl.data == 1:
            rightV = - (speed + (0.5 * speed))
            leftV = - (speed - (0.5 * speed))
         #   rospy.loginfo("im here at left joint ")

        elif right_siganl.data == 1:
            rightV = - (speed - (0.5 * speed))
            leftV = - (speed + (0.5 * speed))
         #   rospy.loginfo("im here at right joint ")

        elif mid_siganl.data == 1:
            leftV = -speed
            rightV = -speed
            flagstart = 1
         #   rospy.loginfo("im here  at middle ")

        if (mid_siganl.data or right_siganl.data or left_siganl.data) == 0:
           # rospy.loginfo("im not in the right place ")
            leftV = speed
            rightV = speed

        if  (mid_siganl.data and right_siganl.data and left_siganl.data) ==  1 :

                    rospy.loginfo("i will stop now")
                    stopsign = stopsign +1
                    leftV=0
                    rightV = 0
                    LWS_pub.publish(Float32(leftV)) 
                    RWS_pub.publish(Float32(rightV))
                    time.sleep(4)
                    rightV = -speed
                    rightV = -speed
                    if(stopsign == 1):
                       rospy.loginfo("this is the first stop , welcome to the first table  ") 
                       
                    if(stopsign == 2):
                       rospy.loginfo("this is the second stop , welcome to the second table ") 
                       
                    if(stopsign == 3):
                       rospy.loginfo("this is the third stop , welcome to the third table ") 
                      
                    if(stopsign == 4):
                       rospy.loginfo("this is the fourth stop , welcome to the fourth table ") 
                    LWS_pub.publish(Float32(leftV)) 
                    RWS_pub.publish(Float32(rightV))
                    time.sleep(2)
                    

        LWS_pub.publish(Float32(leftV))
        RWS_pub.publish(Float32(rightV))
        rate.sleep()



        





            