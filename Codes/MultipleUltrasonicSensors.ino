#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>

#include <Ultrasonic.h>


/*--------------------creating node_handler---------------*/
ros::NodeHandle  Arduino2Handler;
/*-------------------------------------------------------*/

/*-------------------creating_publisher_handlers-----------*/
std_msgs::Int32 front_sensor_msg;  
ros::Publisher front_ultrasonic("front_sensor", &front_sensor_msg);

std_msgs::Int32 right_sensor_msg;
ros::Publisher right_ultrasonic("right_sensor", &right_sensor_msg);

std_msgs::Int32 left_sensor_msg;
ros::Publisher left_ultrasonic("left_sensor", &left_sensor_msg);
 /*----------------------------------------------------------*/
 

Ultrasonic ultrasonic1(12, 13);	// An ultrasonic sensor HC-04
Ultrasonic ultrasonic2(10);		// An ultrasonic sensor PING)))
Ultrasonic ultrasonic3(8);		// An Seeed Studio ultrasonic sensor

/*----------------------------------------------------------*/


void setup() {
  Arduino2Handler.initNode();
  Arduino2Handler.advertise(front_ultrasonic);
  Arduino2Handler.advertise(right_ultrasonic);
  Arduino2Handler.advertise(left_ultrasonic);
  
  Serial.begin(9600);
}

void loop() {
  Serial.print("Sensor 01: ");
  Serial.print(ultrasonic1.read()); // Prints the distance on the default unit (centimeters)
  Serial.println("cm");

  Serial.print("Sensor 02: ");
  Serial.print(ultrasonic2.read(CM)); // Prints the distance making the unit explicit
  Serial.println("cm");

  Serial.print("Sensor 03: ");
  Serial.print(ultrasonic3.read(INC)); // Prints the distance in inches
  Serial.println("inc");

  front_ultrasonic.publish(&front_sensor_msg)
  right_ultrasonic.publish(&right_sensor_msg)
  left_ultrasonic.publish(&left_sensor_msg)

  delay(1000);
}
