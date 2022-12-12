#include <ros.h>

#include <filters.h>
#include <filters_defs.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/Point.h>


#include "PID_v1.h"
#include "TimerOne.h"


#define KP1 10
#define KI1 17
#define KD1 5

#define KP2 10
#define KI2 17
#define KD2 5

//ENCODER
#define ENCODER1_PIN 2
#define ENCODER2_PIN 3
#define ENCODER_PULSE_PER_REV 400


//MOTOR 1
#define MOTOR_Left_A1_PIN 7
#define MOTOR_Left_B1_PIN 8

//MOTOR 2
#define MOTOR_Right_A2_PIN 9
#define MOTOR_Right_B2_PIN 4

#define PWM_MOTOR_1 5
#define PWM_MOTOR_2 6

#define EN_PIN_1 A0
#define EN_PIN_2 A1

// Timer specific macros
#define TIMER_PERIOD_MS 5

/*----------Function_Prototypes--------------*/

void speed_setpoints(const geometry_msgs::Point &set_points);



/*-------------------------------------------*/


/*-------------------Global_variables_decleration---------------*/
double usSpeed1 = 0;
double usSpeed2 = 0;

double max_pwm2=0;
double min_pwm2=0;

double mapped_pwm2=0;
double rpm1;
double rpm2;

double rpm2_filtered=0;
double rpm1_filtered=0;

int counter1;
int counter2;
 double speed_setpoint_right= 0;
 double speed_setpoint_left = 0;

IIR::ORDER  order  = IIR::ORDER::OD2; // Order (OD1 to OD4)
    
PID motor1_PID(&rpm1_filtered,&usSpeed1, &speed_setpoint_left, KP1, KI1, KD1, DIRECT); 
PID motor2_PID(&rpm2_filtered,&usSpeed2, &speed_setpoint_right, KP2, KI2, KD2, DIRECT); 
Filter f1(1, 0.005, order);
Filter f2(1, 0.005, order);

/*--------------------------------------------------------*/



/*--------------------creating node_handler---------------*/
ros::NodeHandle  nh;
/*-------------------------------------------------------*/



/*---------------------creating_subrscriber_handlers-----------*/
ros::Subscriber<geometry_msgs::Point> set_speed_sub("set_points", speed_setpoints);

/*-------------------------------------------------------------*/

/*-------------------creating_publisher_handlers-----------*/
geometry_msgs::Point actual_speeds;  
ros::Publisher actual_speeds_publisher("actual_velocity_publisher", &actual_speeds);


 /*----------------------------------------------------------*/

 

/*--------------callback_functions to define setpoints---------*/
void speed_setpoints( const geometry_msgs::Point &speed_setting){
  
    nh.loginfo("ay haga");
    speed_setpoint_right=(speed_setting.x);
    speed_setpoint_left=(speed_setting.y);

  /*----------------------------------------*/  
  if(speed_setpoint_right<0)
  {
    
    digitalWrite(MOTOR_Right_A2_PIN,1);
    digitalWrite(MOTOR_Right_B2_PIN,0);
  
  }
  else
  {
    //nh.loginfo("uu");
    digitalWrite(MOTOR_Right_A2_PIN, 0);
    digitalWrite(MOTOR_Right_B2_PIN, 1);
    //analogWrite(PWM_MOTOR_1, usSpeed1);

  }
  /*---------------------------------*/
   
   if(speed_setpoint_left<0)
  {
    
    digitalWrite(MOTOR_Left_A1_PIN, 0);
    digitalWrite(MOTOR_Left_B1_PIN,1 );
  
  }
  else
  {
    digitalWrite(MOTOR_Left_A1_PIN,1);
    digitalWrite(MOTOR_Left_B1_PIN,0 );
    //analogWrite(PWM_MOTOR_2, usSpeed2);
  }

  speed_setpoint_right=abs(speed_setting.x);
  speed_setpoint_left=abs(speed_setting.y);
  
}
 



/*--------------------------------------------------------*/


/*-------------Function_ISRS------------------------------*/

void encoder1_ISR(){
  counter1++;
  //Serial.println("Encoder1");
}

void encoder2_ISR(){
  counter2++;
  //Serial.println("Encoder2");
}

void timerISR(){
  
  detachInterrupt(digitalPinToInterrupt(ENCODER1_PIN));
  detachInterrupt(digitalPinToInterrupt(ENCODER2_PIN));
  
//  Serial.print("Counter: ");
//  Serial.println(counter2);
  
  rpm1 = ((double(counter1) / ENCODER_PULSE_PER_REV)*200)*60;
  rpm2 = ((double(counter2) / ENCODER_PULSE_PER_REV)*200)*60;



  //right_motor_actual_speed.publish(&right_wheel_actual_speed_msg);
  //left_motor_actual_speed.publish(&left_wheel_actual_speed_msg);


  //Serial.print("Motor two RPM:");
  //Serial.println(rpm1);
 
  counter1 = 0;     // Reset the counter
  counter2 = 0;     // Reset the counter

  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN), encoder1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN), encoder2_ISR, RISING);
}
/*----------------------------------------------------------------*/


void setup() {


  nh.initNode();
  nh.advertise(actual_speeds_publisher);
  nh.subscribe(set_speed_sub);

  
  pinMode(MOTOR_Left_A1_PIN, OUTPUT);
  pinMode(MOTOR_Left_B1_PIN, OUTPUT);

  pinMode(MOTOR_Right_A2_PIN, OUTPUT);
  pinMode(MOTOR_Right_B2_PIN, OUTPUT);

  pinMode(PWM_MOTOR_1, OUTPUT);
  pinMode(PWM_MOTOR_2, OUTPUT);

  pinMode(EN_PIN_1, OUTPUT);
  pinMode(EN_PIN_2, OUTPUT);

  pinMode(ENCODER1_PIN, INPUT);
  pinMode(ENCODER2_PIN, INPUT);

/*------------------------------------------------*/

//  mapped_pwm2=map(speed_setpoint2,0,240,0,255);
//  max_pwm2=mapped_pwm2-22;
//  min_pwm2=max_pwm2*0.75;
//
//  motor2_PID.SetOutputLimits(min_pwm2,max_pwm2);
//  
//  mapped_pwm2=map(speed_setpoint,0,240,0,255);
//  max_pwm2=mapped_pwm2-25;
//  min_pwm2=max_pwm2*0.75;
//  
//  motor1_PID.SetOutputLimits(min_pwm2,max_pwm2);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_PIN), encoder1_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_PIN), encoder2_ISR, RISING);
  Timer1.initialize(5000);         // initialize timer1, and set a 1/2 second period
  Timer1.attachInterrupt(timerISR);
  motor1_PID.SetMode(AUTOMATIC);
  motor2_PID.SetMode(AUTOMATIC);
  digitalWrite(MOTOR_Left_A1_PIN, 1);
  digitalWrite(MOTOR_Left_B1_PIN, 0);
  digitalWrite(MOTOR_Right_A2_PIN, 0);
  digitalWrite(MOTOR_Right_B2_PIN, 1);

  digitalWrite(EN_PIN_1, HIGH);
  digitalWrite(EN_PIN_2, HIGH);

  actual_speeds.z=0;

  Serial.begin(9600);
  //delay(5);

   //setup up of pins 
/*---------------------------------------*/

    
}

void loop() {
   rpm1_filtered = f1.filterIn(rpm1);
 rpm2_filtered = f2.filterIn(rpm2);
 

/*--------------------------------------------------------------*/
mapped_pwm2=map(speed_setpoint_right,0,240,0,255);
  max_pwm2=mapped_pwm2-22;
  min_pwm2=max_pwm2*0.68;

  motor2_PID.SetOutputLimits(min_pwm2,max_pwm2);
  
  mapped_pwm2=map(speed_setpoint_left,0,240,0,255);
  max_pwm2=mapped_pwm2-25;
  min_pwm2=max_pwm2*0.68;
  
  motor1_PID.SetOutputLimits(min_pwm2,max_pwm2);
  
// left_wheel_actual_speed_msg.data=rpm1_filtered;
// right_wheel_actual_speed_msg.data=rpm2_filtered;


/*---------------------------------------------------------------------*/

 motor1_PID.Compute();
 motor2_PID.Compute();


/*---------------------------------------------------------------------*/
 if(digitalRead(MOTOR_Right_A2_PIN) == 1 && digitalRead(MOTOR_Right_B2_PIN)== 0){ 
   actual_speeds.y=rpm1_filtered;
   actual_speeds.x=rpm2_filtered*-1;
 }
 else if (digitalRead(MOTOR_Left_A1_PIN) == 0 && digitalRead(MOTOR_Left_B1_PIN) == 1){
 actual_speeds.y=rpm1_filtered*-1;
 actual_speeds.x=rpm2_filtered;
  }

else {
   actual_speeds.y=rpm1_filtered;
   actual_speeds.x=rpm2_filtered;
  }

/*-------------------------------------------------------*/

  
 actual_speeds_publisher.publish(&actual_speeds);

 
//publishing actual speeds of the motors
 
//
 analogWrite(PWM_MOTOR_1, usSpeed1);
analogWrite(PWM_MOTOR_2,usSpeed2);

// analogWrite(PWM_MOTOR_1, 90);
// analogWrite(PWM_MOTOR_2,90);

 nh.spinOnce();
 delay(5);

 



 //calculating the speeds of the motor

 /*--------------------------------------------------------------*/
   
//Serial.print("GIVEN SPEED: ");
//Serial.println(usSpeed1);
//Serial.println(60);



/*-----------------------------------------------------------------*/  
// Serial.print("setpoint: ");
// Serial.println(speed_setpoint); 
// Serial.print("MOTOR TWO: ");
// Serial.println(rpm2_filtered);
// Serial.print("MOTOR ONE: ");
// Serial.println(rpm1_filtered);
 //Serial.print(" MOTOR ONE: ");
 //Serial.println(rpm1);


 /*-----------------------------------------------------------------*/

}
