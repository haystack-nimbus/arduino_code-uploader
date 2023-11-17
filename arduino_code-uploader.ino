/*=========================================================================
Filename : cliff_uv_lamp_controller.ino
Project  : Cliff controller
Purpose  : Firmware for arduino uno.
           - to read data from IR sensor
           - to control relay connected to UV Lamp
---------------------------------------------------------------------------
---------------------------------------------------------------------------
Version Number : 1.0
Last Updated   : September 22, 2021
Updated By     : Manodhayan K
---------------------------------------------------------------------------
Copyright (c) 2012 Mobiveil. All rights reserved.
---------------------------------------------------------------------------
This file contains trade secrets of Mobiveil. 
No part may be reproduced or transmitted in any form by any means or for 
any purpose without the express written permission of Mobiveil.
---------------------------------------------------------------------------
Revision History
---------------------------------------------------------------------------


---------------------------------------------------------------------------
Known Issues
---------------------------------------------------------------------------

---------------------------------------------------------------------------
To Do List
---------------------------------------------------------------------------
*/

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>

// PINMAPS
#define UV_LAMP_PIN                         (A0)
#define REAR_LEFT_CLIFF_PIN                 (A1)
#define FRONT_LEFT_CLIFF_PIN                (A2)
#define FRONT_RIGHT_CLIFF_PIN               (A3)
#define REAR_RIGHT_CLIFF_PIN                (A4)


#define FRONT_LEFT_SONAR_ECHO_PIN           (3) 
#define FRONT_LEFT_SONAR_TRIG_PIN           (2) 

#define FRONT_MID_SONAR_ECHO_PIN            (5) 
#define FRONT_MID_SONAR_TRIG_PIN            (4) 

#define FRONT_RIGHT_SONAR_ECHO_PIN          (7) 
#define FRONT_RIGHT_SONAR_TRIG_PIN          (6) 

#define BACK_RIGHT_SONAR_ECHO_PIN           (9) 
#define BACK_RIGHT_SONAR_TRIG_PIN           (8)

#define BACK_MID_SONAR_ECHO_PIN             (11)
#define BACK_MID_SONAR_TRIG_PIN             (10)

#define BACK_LEFT_SONAR_ECHO_PIN            (13)
#define BACK_LEFT_SONAR_TRIG_PIN            (12)

// Constant value to convert analog values from IR sensor to distance in meters
#define A                                   (0.00082712905)
#define B                                   (939.57652)
#define C                                   (-3.3978697)
#define D                                   (17.339222)
#define ADC_BIT                             (10)
#define OPERATING_VOLTAGE                   (5.0)

// Frame IDs
#define FRONT_LEFT_CLIFF_FRAME_ID           ("cliff_front_left")
#define FRONT_RIGHT_CLIFF_FRAME_ID          ("cliff_front_right")
#define REAR_LEFT_CLIFF_FRAME_ID            ("cliff_rear_left")
#define REAR_RIGHT_CLIFF_FRAME_ID           ("cliff_rear_right")
#define UV_LAMP_FRAME_ID                    ("uv_lamp")
#define FRONT_LEFT_SONAR_FRAME_ID           ("sonar_front_left")
#define FRONT_MID_SONAR_FRAME_ID            ("sonar_front_mid")
#define FRONT_RIGHT_SONAR_FRAME_ID          ("sonar_front_right")
#define BACK_RIGHT_SONAR_FRAME_ID           ("sonar_back_right")
#define BACK_MID_SONAR_FRAME_ID             ("sonar_back_mid")
#define BACK_LEFT_SONAR_FRAME_ID            ("sonar_back_left")

// Topics
#define FRONT_LEFT_CLIFF_TOPIC_NAME         ("/cliff_front_left")
#define FRONT_RIGHT_CLIFF_TOPIC_NAME        ("/cliff_front_right")
#define REAR_LEFT_CLIFF_TOPIC_NAME          ("/cliff_rear_left")
#define REAR_RIGHT_CLIFF_TOPIC_NAME         ("/cliff_rear_right")

#define UV_LAMP_STATUS_TOPIC_NAME           ("/uv_lamp_current_status")
#define UV_LAMP_SET_STATUS_TOPIC_NAME       ("/uv_lamp_set_state")

#define FRONT_LEFT_SONAR_TOPIC_NAME         ("/sonar_front_left")
#define FRONT_MID_SONAR_TOPIC_NAME          ("/sonar_front_mid")
#define FRONT_RIGHT_SONAR_TOPIC_NAME        ("/sonar_front_right")
#define BACK_RIGHT_SONAR_TOPIC_NAME         ("/sonar_back_right")
#define BACK_MID_SONAR_TOPIC_NAME           ("/sonar_back_mid")
#define BACK_LEFT_SONAR_TOPIC_NAME          ("/sonar_back_left")


#define CLIFF_FIELD_OF_VIEW                 (0.5) // in radians
#define CLIFF_MIN_RANGE                     (0.2) // in meters
#define CLIFF_MAX_RANGE                     (1.5) // in meters

#define SONAR_FIELD_OF_VIEW                 (1.0) // in radians
#define SONAR_MIN_RANGE                     (0.20) // in meters
#define SONAR_MAX_RANGE                     (3.0) // in meters

#define LOOP_DELAY                          (100)
#define CM_TO_M(X)                          (X/100)
#define FALSE                               (0)
#define TRUE                                (1)
#define DEBUG                               (FALSE)
#define TURN_ON_TIME                        (1 * 1000) // in milliseconds


ros::NodeHandle  nodeHandler;
std_msgs::Bool uvLampStatus_msg;
sensor_msgs::Range Range_msg;

unsigned long range_timer;
unsigned long prevTime;

void uvLampToggleCallback(const std_msgs::Bool& toggle_msg);
// Publishers
ros::Publisher pub_FrontLeftCliff(FRONT_LEFT_CLIFF_TOPIC_NAME, &Range_msg);
ros::Publisher pub_FrontRightCliff(FRONT_RIGHT_CLIFF_TOPIC_NAME, &Range_msg);
ros::Publisher pub_RearLeftCliff( REAR_LEFT_CLIFF_TOPIC_NAME, &Range_msg);
ros::Publisher pub_RearRightCliff(REAR_RIGHT_CLIFF_TOPIC_NAME, &Range_msg);

ros::Publisher pub_UVLampStatus( UV_LAMP_STATUS_TOPIC_NAME, &uvLampStatus_msg);

ros::Publisher pub_FrontLeftSonar(FRONT_LEFT_SONAR_TOPIC_NAME, &Range_msg);
ros::Publisher pub_FrontMidSonar(FRONT_MID_SONAR_TOPIC_NAME, &Range_msg);
ros::Publisher pub_FrontRightSonar(FRONT_RIGHT_SONAR_TOPIC_NAME, &Range_msg);
ros::Publisher pub_BackRightSonar(BACK_RIGHT_SONAR_TOPIC_NAME, &Range_msg);
ros::Publisher pub_BackMidSonar(BACK_MID_SONAR_TOPIC_NAME, &Range_msg);
ros::Publisher pub_BackLeftSonar(BACK_LEFT_SONAR_TOPIC_NAME, &Range_msg);

// Subscribers
ros::Subscriber<std_msgs::Bool> sub_UVLampState(UV_LAMP_SET_STATUS_TOPIC_NAME, &uvLampToggleCallback);


/*============================================================================
Name    :   uvLampToggleCallback()
------------------------------------------------------------------------------
Purpose :   Callback to be called whenever a message is posted on 
            "UV_LAMP_SET_STATUS_PUB_TOPIC_NAME" topic
Input   :   toggle_msg - a boolean message to set the lamp state
Output  :   n/a
Notes   :   n/a
============================================================================*/

void uvLampToggleCallback(const std_msgs::Bool& toggle_msg)
{
  prevTime = millis();
  digitalWrite(UV_LAMP_PIN, toggle_msg.data);
  uvLampStatus_msg.data = toggle_msg.data;
  delay(LOOP_DELAY);
}

/*============================================================================
Name    :   getCliffRange()
------------------------------------------------------------------------------
Purpose :   To read analog value from IR sensor and convert it into distance
Input   :   iPin - pin number to read value from
Output  :   distance measured from IR sensor in meters

Notes   :   Voltage to Distance(cm) conversion formula taken from application note
            Distance = (A + (B * V))/(1 + (C * V) + (D * V * V));
            A = 0.0008271
            B = 939.6
            C = -3.398
            D = 17.339
            V = Voltage
============================================================================*/

float getCliffRange(int iPin)
{
    
    float dVoltage;
    float dDistance;
    dVoltage = (float) OPERATING_VOLTAGE * (analogRead(iPin) / (float) (1 << ADC_BIT));
    dDistance = (A + (B * dVoltage))/(1 + (C * dVoltage) + (D * dVoltage * dVoltage));
    return CM_TO_M(dDistance);
    
}

/*============================================================================
Name    :   getSonarRange(int trigPin, int echoPin)
------------------------------------------------------------------------------
Purpose :   It is used to calculate the distane of the obastacle
Input   :   trigger and echo pin of the ultrasound sensor
Output  :   n/a
Notes   :   n/a
============================================================================*/

float getSonarRange(int trigPin, int echoPin){
  
  // Clears the trigPin condition
  float distance;
  float duration;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  return CM_TO_M(distance);
 
}

/*============================================================================
Name    :   setup()
------------------------------------------------------------------------------
Purpose :   Main function where the program begins
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/

void setup()
{

  // Pin Initialization
  pinMode(FRONT_LEFT_CLIFF_PIN, INPUT);
  pinMode(FRONT_RIGHT_CLIFF_PIN, INPUT);
  pinMode(REAR_LEFT_CLIFF_PIN, INPUT);
  pinMode(REAR_RIGHT_CLIFF_PIN, INPUT);

  pinMode(UV_LAMP_PIN, OUTPUT);

  pinMode(FRONT_LEFT_SONAR_TRIG_PIN, OUTPUT);
  pinMode(FRONT_MID_SONAR_TRIG_PIN, OUTPUT);
  pinMode(FRONT_RIGHT_SONAR_TRIG_PIN, OUTPUT);
  pinMode(BACK_RIGHT_SONAR_TRIG_PIN, OUTPUT);
  pinMode(BACK_MID_SONAR_TRIG_PIN, OUTPUT);
  pinMode(BACK_LEFT_SONAR_TRIG_PIN, OUTPUT);
 
  pinMode(FRONT_LEFT_SONAR_ECHO_PIN, INPUT);
  pinMode(FRONT_MID_SONAR_ECHO_PIN, INPUT);
  pinMode(FRONT_RIGHT_SONAR_ECHO_PIN, INPUT);
  pinMode(BACK_RIGHT_SONAR_ECHO_PIN, INPUT);
  pinMode(BACK_MID_SONAR_ECHO_PIN, INPUT);
  pinMode(BACK_LEFT_SONAR_ECHO_PIN, INPUT);
 

  // Init ros node
  nodeHandler.initNode();

  // Advertise the topic to be published
  nodeHandler.advertise(pub_FrontLeftCliff);
  nodeHandler.advertise(pub_FrontRightCliff);
  nodeHandler.advertise(pub_RearLeftCliff);
  nodeHandler.advertise(pub_RearRightCliff);
  nodeHandler.advertise(pub_UVLampStatus);

  // Initiate subscription
  nodeHandler.subscribe(sub_UVLampState);

  nodeHandler.advertise(pub_FrontLeftSonar);
  nodeHandler.advertise(pub_FrontMidSonar);
  nodeHandler.advertise(pub_FrontRightSonar);
  nodeHandler.advertise(pub_BackRightSonar);
  nodeHandler.advertise(pub_BackMidSonar);
  nodeHandler.advertise(pub_BackLeftSonar);

  uvLampStatus_msg.data = FALSE;

}

/*============================================================================
Name    :   loop()
------------------------------------------------------------------------------
Purpose :   Continously read sensor data & publish uv lamp status
Input   :   n/a
Output  :   n/a
Notes   :   n/a
============================================================================*/


void loop()
{

  if ( (millis() - range_timer) > LOOP_DELAY){

    // Cliff Default Values
    Range_msg.field_of_view = CLIFF_FIELD_OF_VIEW;
    Range_msg.min_range = CLIFF_MIN_RANGE;
    Range_msg.max_range = CLIFF_MAX_RANGE;

    // Cliff Front Left
    Range_msg.header.stamp = nodeHandler.now();
    Range_msg.range = getCliffRange(FRONT_LEFT_CLIFF_PIN);
    Range_msg.radiation_type = sensor_msgs::Range::INFRARED;
    Range_msg.header.frame_id = FRONT_LEFT_CLIFF_FRAME_ID;
    pub_FrontLeftCliff.publish(&Range_msg);
  
    // Cliff Front Right
    Range_msg.header.stamp = nodeHandler.now();
    Range_msg.range = getCliffRange(FRONT_RIGHT_CLIFF_PIN);
    Range_msg.header.frame_id = FRONT_RIGHT_CLIFF_FRAME_ID;
    pub_FrontRightCliff.publish(&Range_msg);
   
    // Cliff Rear Left
    Range_msg.header.stamp = nodeHandler.now();
    Range_msg.range = getCliffRange(REAR_LEFT_CLIFF_PIN);
    Range_msg.header.frame_id = REAR_LEFT_CLIFF_FRAME_ID;
    pub_RearLeftCliff.publish(&Range_msg);
   

    // Cliff Rear right
    Range_msg.header.stamp = nodeHandler.now();
    Range_msg.range = getCliffRange(REAR_RIGHT_CLIFF_PIN);
    Range_msg.header.frame_id = REAR_RIGHT_CLIFF_FRAME_ID;
    pub_RearRightCliff.publish(&Range_msg);

     // Sonar Default Values
    Range_msg.field_of_view = SONAR_FIELD_OF_VIEW;
    Range_msg.min_range = SONAR_MIN_RANGE;
    Range_msg.max_range = SONAR_MAX_RANGE;

    // Sonar Front Left
    Range_msg.header.stamp = nodeHandler.now();
    Range_msg.range = getSonarRange(FRONT_LEFT_SONAR_TRIG_PIN, FRONT_LEFT_SONAR_ECHO_PIN);
    Range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    Range_msg.header.frame_id = FRONT_LEFT_SONAR_FRAME_ID;
    pub_FrontLeftSonar.publish(&Range_msg);

    // Sonar Front Mid
    Range_msg.header.stamp = nodeHandler.now();
    Range_msg.range = getSonarRange(FRONT_MID_SONAR_TRIG_PIN, FRONT_MID_SONAR_ECHO_PIN);
    Range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    Range_msg.header.frame_id = FRONT_MID_SONAR_FRAME_ID;
    pub_FrontMidSonar.publish(&Range_msg);
    
    // Sonar Front Right
    Range_msg.header.stamp = nodeHandler.now();
    Range_msg.range = getSonarRange(FRONT_RIGHT_SONAR_TRIG_PIN, FRONT_RIGHT_SONAR_ECHO_PIN);
    Range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    Range_msg.header.frame_id = FRONT_RIGHT_SONAR_FRAME_ID;
    pub_FrontRightSonar.publish(&Range_msg);

    // Sonar Back Right
    Range_msg.header.stamp = nodeHandler.now();
    Range_msg.range = getSonarRange(BACK_RIGHT_SONAR_TRIG_PIN, BACK_RIGHT_SONAR_ECHO_PIN);
    Range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    Range_msg.header.frame_id = BACK_RIGHT_SONAR_FRAME_ID;
    pub_BackRightSonar.publish(&Range_msg);

    // Sonar Back Mid
    Range_msg.header.stamp = nodeHandler.now();
    Range_msg.range = getSonarRange(BACK_MID_SONAR_TRIG_PIN, BACK_MID_SONAR_ECHO_PIN);
    Range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    Range_msg.header.frame_id = BACK_MID_SONAR_FRAME_ID;
    pub_BackMidSonar.publish(&Range_msg);

    // Sonar Back Left
    Range_msg.header.stamp = nodeHandler.now();
    Range_msg.range = getSonarRange(BACK_LEFT_SONAR_TRIG_PIN, BACK_LEFT_SONAR_ECHO_PIN);
    Range_msg.header.frame_id = BACK_LEFT_SONAR_FRAME_ID;
    pub_BackLeftSonar.publish(&Range_msg);

    
    if ((millis() - prevTime)> TURN_ON_TIME)
    {

      uvLampStatus_msg.data = false;
      uvLampToggleCallback(uvLampStatus_msg);
    }

    // UV Lamp status
    pub_UVLampStatus.publish(&uvLampStatus_msg);

    range_timer =  millis();
  }
  nodeHandler.spinOnce();
}
