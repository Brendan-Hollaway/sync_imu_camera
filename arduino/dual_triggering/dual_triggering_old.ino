#include <ros.h>
#include <std_msgs/Byte.h>

//ros::NodeHandle nh;

//std_msgs::Byte byte_msg;
//ros::Publisher serialdata("serialdata", &byte_msg);


//Defines the pins of each sensor's Sync_In pin
const int camera_pin = 3;
const int imu_pin = 2;
const int manual_offset = 3 * 1000; //offset in micros (camera - imu)

//Sets the rate of triggering of each sensor
const int max_camera_rate = 2; //Hz. Must be a divisor of max_imu_rate. Must be <= max_imu_rate
// Target: 800. Actual: 400-405
// Target: 400. Actual: 384
// Target: 390. Actual: 377
// Target: 360. Actual: 355
// Target: 340. Actual: 338.4
// Target: 338. ACtual: 335.5
// Target: 335. Actual: 332.8
// Target: 330. Actual: 329.6
// The error on 330 is low enough that there are not sync issues.
// 0-62: -1. 63-125: -2. 126-187: -3.
//16129.032258065 - 62
//15873.015873016 - 63
//8000            - 125
//7936.507936508  - 126

const int max_imu_rate = 188; //Hz. Minimum: 129
const int imu_camera_ratio = max_imu_rate / max_camera_rate;

float previousTimeImu = 0;
float previousTimeCamera = 0;
//int num_imu_ticks_since_last_photo = 1;
float currentTime;

//Time between triggers based on rates (in micros)
const float camera_interval = 1000000.0 / max_camera_rate;
const float imu_interval = 1000000.0 / max_imu_rate;

const float offset = manual_offset + imu_interval * (imu_camera_ratio - 3);

//Triggers each sensor
void sensorTriggering(int pinnumber) {
  //Serial.write(currentTime);
  digitalWrite(pinnumber, HIGH);
  delayMicroseconds(100);
  digitalWrite(pinnumber, LOW);
}

// Triggers both sensors by sending a 100 microsecond long square wave
// to their digital pins
void triggerBothSensors() {
  //Serial.write(currentTime);

  // PORTD represents the arduino digital ports 0-7 in bits 0-7.
  // Setting these two bits to true will thus set those pins to HIGH
  PORTD |= (1 << camera_pin) | (1 << imu_pin);

  delayMicroseconds(100);

  // Setting these two bits to false will thus set those pins to FALSE
  PORTD ^= (1 << camera_pin) | (1 << imu_pin);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  //  nh.initNode();
  //  nh.advertise(serialdata);
  pinMode(camera_pin, OUTPUT);
  pinMode(imu_pin, OUTPUT);
  //  delay(7 * 1000); // delay 7s to allow for setup time.
  // Set up previous times to trigger camera instantly,
  // and imu at the appropriate time.
  currentTime = micros();
  previousTimeCamera = micros() - camera_interval;
  previousTimeImu = previousTimeCamera + offset;
}

void loop() {

  currentTime = micros();



  //  if (num_imu_ticks_since_last_photo == imu_camera_ratio &&
  //      currentTime - previousTimeImu >= imu_interval) {
  //    triggerBothSensors();
  ////    previousTimeCamera = currentTime;
  ////    previousTimeImu = currentTime;
  //    previousTimeImu += imu_interval;
  //    num_imu_ticks_since_last_photo = 1;
  //  }



  //  else
  if (currentTime - previousTimeImu >= imu_interval) {
    sensorTriggering(imu_pin);
    previousTimeImu += imu_interval;
  }

  if (currentTime - previousTimeCamera >= camera_interval) {
    sensorTriggering(camera_pin);
//    Serial.print(" Change in imu time ");
//    Serial.println((previousTimeCamera + offset - previousTimeImu);
    //    Serial.print(" Current cam time ");
    //    Serial.println(previousTimeCamera);


    // The last time the IMU should have triggered should be 
    // the same as the last time the camera triggered, plus 
    // the time required for the IMU to trigger one full cycle of times,
    // plus the offset.
    previousTimeImu = previousTimeCamera + offset;
    previousTimeCamera += camera_interval;
    //    Serial.print(" New imu time ");
    //    Serial.println(previousTimeImu);
    //
    //    Serial.print(" Current time: ");
    //    Serial.println(currentTime);

  }


  /*
    if (currentTime - previousTimeCamera >= camera_interval) {
    sensorTriggering(camera_pin);
    previousTimeCamera = currentTime;
    }

    if (currentTime - previousTimeImu >= imu_interval) {
    sensorTriggering(imu_pin);
    previousTimeImu = currentTime;
    }*/
  //
  //  if (Serial.available() > 0) {
  //    byte data = Serial.read();
  //    Serial.print(data);
  //  }
  //
//    nh.spinOnce();
}
