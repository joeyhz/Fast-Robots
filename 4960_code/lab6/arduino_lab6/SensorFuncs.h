#ifndef SENSOR_FUNCS_H
#define SENSOR_FUNCS_H

#include <ComponentObject.h>
#include <RangeSensor.h>
#include <SparkFun_VL53L1X.h>
#include <vl53l1x_class.h>
#include <vl53l1_error_codes.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

//TOF1 interrupt and shutdown pins.
#define SHUTDOWN_PIN 8
#define INTERRUPT_PIN 3


//Uncomment the following line to use the optional shutdown and interrupt pins.
SFEVL53L1X distanceSensor1(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distanceSensor2;

void setup_sensors(){
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

bool sensor_data_ready(){
  return distanceSensor2.checkForDataReady();
}

int get_front_tof(){
  if(!sensor_data_ready()) return -1000; //should never happen
  
  int distance2 = distanceSensor2.getDistance(); //Get the result of the measurement from sensor 2
  distanceSensor2.clearInterrupt();
  distanceSensor2.stopRanging();
  
  Serial.print("Distance 2(mm): ");
  Serial.print(distance2);
  
  return distance2;

}
#endif
