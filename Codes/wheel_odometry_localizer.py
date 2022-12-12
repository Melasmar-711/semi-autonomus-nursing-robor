import rospy
import math
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Point

WHEEL_RADIUS = 0.0325
WHEEL_DISTANCE = 0.1334*2

def pi_2_pi(angle):
    # a function to wrap the angle between -pi and pi (for numerical stability)
    return (angle + math.pi) % (2 * math.pi) - math.pi

def forward_model(wl,wr):
    # units are m/s and rad/s
    xdot = ((wl+wr)*WHEEL_RADIUS)/2
    thetadot = ((wr-wl)*WHEEL_RADIUS)/WHEEL_DISTANCE
    return xdot, thetadot

def wheel_speeds_callback(msg):
    global wl, wr
    wl = ((msg.y)*2*math.pi)/60
    wr = ((msg.x)*2*math.pi)/60
    rospy.loginfo("clbk")


if __name__ == "__main__":

    # Initialize the ROS node
    rospy.init_node("wheel_odometry_localizer_node")

    # Initialize wheel speed values 
    wl, wr = 0, 0

    # Publisher for the pose topic 
    pose_pub = rospy.Publisher("/wheel_odometry_localizer/pose", Point, queue_size= 10)

    # Subscriber to the velocity commanding topic
    rospy.Subscriber("actual_velocity_publisher", Point, wheel_speeds_callback)
    #rospy.Subscriber("right_actual_speed", Float32, right_wheel_speed_callback)

    # Initialize time for integration
    t_start = rospy.get_time()
    t_prev = rospy.get_time()


    # Initialize variables for trapezoidal integration
    xdot_prev, thetadot_prev = 0, 0

    # Initialize integration values 
    x, y, theta = 0, 0, 0

    rate = rospy.Rate(30) # 10 Hz
    rospy.loginfo("Connecting to CoppeliaSim topics")
    rospy.loginfo("Accessing wheel speeds")
    
    while not rospy.is_shutdown():

        xdot, thetadot = forward_model(wl, wr)

        # calculate dt
        t_start =  rospy.get_time()
        dt = t_start - t_prev


        # do the integration (trapezoidal)
        x += ((xdot + xdot_prev) * (math.cos(theta)) * dt / 2)

        y += ((xdot + xdot_prev) * (math.sin(theta)) * dt / 2)
        theta += ((thetadot + thetadot_prev) * dt / 2)
        theta = pi_2_pi(theta) # wrap-to-pi
        t_prev = t_start
        
        # equate variables for next iteration
        xdot_prev = xdot
        thetadot_prev = thetadot

        # initialize ros message to be published


        rospy.loginfo("X:%s",x)
        pose_pub.publish(x,y,theta)

        rate.sleep()

        
    rospy.loginfo("Robot localization finished or terminated!")
