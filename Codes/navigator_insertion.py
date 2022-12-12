import rospy
import math
import time
from std_msgs.msg import Float32, Float32MultiArray, Int32
from geometry_msgs.msg import Point

WHEEL_RADIUS = 0.0325
WHEEL_DISTANCE = 0.1334*2

#DIST_THRESHOLD = 0.05
#YAW_THRESHOLD = 0.05
#THETADOT_NAV = 0.5
#XDOT_NAV = 0.2

DIST_THRESHOLD = 0.1 
YAW_THRESHOLD =0.5
THETADOT_NAV = 1.3
XDOT_NAV = 0.2

TRAJECTORY=[
    [0.5, 0],
    [0.5, 0],
    [1,   0],
    [1.5, 0],
    [2,   0],
    [2.5, 0],
    [3,  0],
    [3.5, 0],
    [5,   0],
    [5.5,0],
    [5.7, 0],
    [8, 0],
    [11,0],
    [12, -1.5],

]


def inverse_model(xdot_, thetadot_):
    # units are m/s and rad/s
    wl = ((xdot_ - (thetadot_*WHEEL_DISTANCE/2)) / WHEEL_RADIUS)
    wr = ((xdot_ + (thetadot_*WHEEL_DISTANCE/2)) / WHEEL_RADIUS)
    return wl, wr

def pose_clbk(msg):
    global x, y, theta
    x = msg.x
    y = msg.y
    theta = msg.z
    rospy.loginfo("CLBK")
    

def euclidean_distance(x_, y_, goal_point_):
    dist_x = goal_point_[0] - x_
    dist_y = goal_point_[1] - y_
    dist = math.sqrt(dist_x**2 + dist_y**2)
    return dist

def sign(x):
    if x >= 0:
        return 1
    else: 
        return -1

def pi_2_pi(angle):
    # a function to wrap the angle between -pi and pi (for numerical stability)
    return (angle + math.pi) % (2 * math.pi) - math.pi

def steering_angle(x_, y_, theta_, goal_point_):

    return (math.atan2( (goal_point_[1] - y_) , (goal_point_[0] - x_) ) - theta_)

if __name__ == "__main__":

    rospy.init_node("navigator_node")

    # Initialize pose values
    x, y, theta = 0, 0, 0

    # Subscriber to the velocity commanding topic
    rospy.Subscriber("/wheel_odometry_localizer/pose", Point, pose_clbk)
    
    # Setup wheel speed publishers
    wheel_speeds_pub = rospy.Publisher("set_points", Point, queue_size=10)
    #right_wheel_speed_pub = rospy.Publisher("right_setpoint_speed", Float32, queue_size=10)
    table_motor = rospy.Publisher("/sim_ros_interface/servo_right", Int32, queue_size=10)
    # Setup ROS rate 
    rate = rospy.Rate(10) # 10 Hz
    rospy.loginfo("Navigation node successfully spawned")

    while not rospy.is_shutdown():

        rospy.loginfo("Starting to traverse the path")
        TRAJECTORY_ORIGINAL = TRAJECTORY

        for goal_point in range(100):
        #    if (TRAJECTORY[goal_point][0]==5.8)and(TRAJECTORY[goal_point][1]==0):
                
                #while(1):
                       #left_wheel_speed_pub.publish(Float32(5))
                       #right_wheel_speed_pub.publish(Float32(-5))
                       #rospy.loginfo(theta)
             #   for i in range(10000):
             #          left_wheel_speed_pub.publish(Float32(5))
             #          right_wheel_speed_pub.publish(Float32(-5))
            while euclidean_distance(x, y, TRAJECTORY[goal_point]) > DIST_THRESHOLD and not rospy.is_shutdown():
                rospy.loginfo("Euclidean_Dist: %s", euclidean_distance(x, y, TRAJECTORY[goal_point]))

                # # Read proximity sensor
                # msg_front_sensor=1
                rospy.loginfo(TRAJECTORY[goal_point])
                rospy.loginfo("X:%s     Y:%s", x, y)
                rospy.loginfo("Theta:%s",theta)

                #msg_front_sensor = rospy.wait_for_message("/sim_ros_interface/front/state", Int32)
                
                #y_new = y + 1
                #TRAJECTORY.insert(goal_point,[x,y_new])
                #if msg_front_sensor.data == 1:
                       #rospy.loginfo("Front")
                       #msg_left_sensor = rospy.wait_for_message("/sim_ros_interface/proximity_sensor_left/state", Int32)
                       #rospy.loginfo(f"left data{msg_left_sensor.data}")
                       #msg_right_sensor = rospy.wait_for_message("/sim_ros_interface/proximity_sensor_right/state", Int32)
                       #rospy.loginfo(f"right data{msg_right_sensor.data}")
                	
                       #if msg_left_sensor.data == 0:
                	
                               #y_new = y + 1
                               #TRAJECTORY.insert(goal_point,[x,y_new])
				# x_new = x + 0.5
				#TRAJECTORY.remove(TRAJECTORY[goal_point])
				#TRAJECTORY.insert(goal_point+2,TRAJECTORY[goal_point])
                 
                    		#TRAJECTORY[goal_point] = [x,y_new]
                    		
                       #elif msg_right_sensor.data == 0:
                       	
                               #rospy.loginfo("TURNING RIGHT")
                               #y_new = y -0.8
                               #TRAJECTORY.insert(goal_point,[x,y_new])
                    		
                # Safe to navigate

                if abs(steering_angle(x, y, theta, TRAJECTORY[goal_point])) > YAW_THRESHOLD:
                    rospy.loginfo("changing angle")
                    xdot = 0
                    thetadot = THETADOT_NAV * sign (steering_angle(x, y, theta, TRAJECTORY[goal_point]))
                    # if thetadot < 0.01:
                    #     thetadot = 0

                else:
                    rospy.loginfo("changing speed")
                    rospy.loginfo("NOW")
                    xdot = XDOT_NAV
                    thetadot = 0

                rospy.loginfo("thetadot: %s",thetadot)
                lw_speed, rw_speed = inverse_model(xdot, thetadot)

                wheel_speeds_pub.publish(rw_speed*60/(2*math.pi),(lw_speed*60/(2*math.pi)),0)
                #wheel_speeds_pub.publish(90,90,0)
                #right_wheel_speed_pub.publish(Float32(rw_speed*60/(2*math.pi)))
                #left_wheel_speed_pub.publish(70)
                #right_wheel_speed_pub.publish(70)
                rospy.loginfo("Ra!")

                rate.sleep()

            if (goal_point==(len(TRAJECTORY))-1):
                break

        wheel_speeds_pub.publish(0,0,0)
        time.sleep(5)
        table_motor.publish(0)
        rospy.loginfo("Robot navigation finished or terminated!")
        TRAJECTORY = TRAJECTORY_ORIGINAL
        break
        
