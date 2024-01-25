#include <ArloRobot.h>    // Include Arlo library
#include <SoftwareSerial.h> // Include SoftwareSerial library
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>


// Arlo and serial objects required 
ArloRobot Arlo;     // Arlo object
SoftwareSerial ArloSerial (25, 26); // Serial in I/O 12, out I/O 13

const int ticksMeter = 300;


//int countsLeft, countsRight;  // Encoder counting variables

float leftSpeed, rightSpeed;

ros::NodeHandle nh;
std_msgs::Int16 countsLeft;
std_msgs::Int16 countsRight;

void
leftSpeedCb (const std_msgs::Float32 & speed_msg)
{
  leftSpeed = speed_msg.data * ticksMeter;
}

void
rightSpeedCb (const std_msgs::Float32 & speed_msg)
{
  rightSpeed = speed_msg.data * ticksMeter;
}

//Publishers
ros::Publisher lwheelPub ("lwheel", &countsLeft);
ros::Publisher rwheelPub ("rwheel", &countsRight);

//Subscribers
ros::Subscriber < std_msgs::Float32 > lwheelSpeedSub ("/lwheel_vtarget",
                  &leftSpeedCb);
ros::Subscriber < std_msgs::Float32 > rwheelSpeedSub ("/rwheel_vtarget",
                  &rightSpeedCb);

void
setup ()      // Setup function
{
  Serial.begin (9600);    // Start terminal serial port
  //Serial2.begin(19200);
  ArloSerial.begin (19200); // Start DHB-10 serial com
  Arlo.begin (ArloSerial);  // Pass to Arlo object

  Arlo.clearCounts ();    // Clear encoder counts

  nh.initNode ();
  nh.advertise (lwheelPub);
  nh.advertise (rwheelPub);
  nh.subscribe (lwheelSpeedSub);
  nh.subscribe (rwheelSpeedSub);

}

void
loop ()
{
  Arlo.writeSpeeds (leftSpeed, rightSpeed);
  countsLeft.data = Arlo.readCountsLeft ();
  countsRight.data = Arlo.readCountsRight ();
  lwheelPub.publish (&countsLeft);
  rwheelPub.publish (&countsRight);
  nh.spinOnce();
  delay(50);
}
