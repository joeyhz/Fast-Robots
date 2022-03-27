#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>

/*
  Reading distance from the laser based VL53L1X
  By: Nathan Seidle
  SparkFun Electronics
  Date: April 4th, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/14667

  This example prints the distance to an object.

  Are you getting weird readings? Be sure the vacuum tape has been removed from the sensor.
*/

#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//Optional interrupt and shutdown pins.
#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3


//Uncomment the following line to use the optional shutdown and interrupt pins.
SFEVL53L1X distanceSensor1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distanceSensor2;

void setup(void)
{
  Serial.begin(115200);
  Serial.println("start");


  Wire.begin();

  Serial.println("VL53L1X Qwiic Test");

  if (distanceSensor2.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor 2 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor 2 online!");

  digitalWrite(SHUTDOWN_PIN, LOW); // Shut off ToF 1
  distanceSensor2.setI2CAddress(0x22);
  digitalWrite(SHUTDOWN_PIN, HIGH); // Shut off ToF 1

  Serial.println("changed address");

  if (distanceSensor1.begin() != 0) //Begin returns 0 on a good init
  {
    Serial.println("Sensor 1 failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor 1 online!");
}

void loop(void)
{
  distanceSensor1.startRanging(); //Write configuration bytes to initiate measurement
  distanceSensor2.startRanging();
  Serial.println("ranging");
  while ( !(distanceSensor1.checkForDataReady() && distanceSensor2.checkForDataReady()) )
  {
    delay(1);
  }
  int distance1 = distanceSensor1.getDistance(); //Get the result of the measurement from sensor 1
  int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from sensor 1

  distanceSensor1.clearInterrupt();
  distanceSensor1.stopRanging();

  distanceSensor2.clearInterrupt();
  distanceSensor2.stopRanging();

  Serial.print("Distance 1(mm): ");
  Serial.print(distance1);
  Serial.println();
  Serial.print("Distance 2(mm): ");
  Serial.print(distance2);


  Serial.println();
}
