#include <ros.h>
#include <std_msgs/UInt64.h>

ros::NodeHandle nh;

// TODO: can we re-use a message?
std_msgs::UInt64 ulong_msg1;
std_msgs::UInt64 ulong_msg2;
ros::Publisher imu_timestamps("imu_timestamps", &ulong_msg1);
ros::Publisher cam_timestamps("cam_timestamps", &ulong_msg2);

//Defines the pins of each sensor's Sync_In pin
const int camera_pin = 3;
const int imu_pin = 2;
/* const int offset = 16 * 1000; //offset in micros (camera - imu) */

//Sets the rate of triggering of each sensor
const int max_camera_rate = 30; //Hz. Must be a divisor of max_imu_rate. Must be <= max_imu_rate

// Target: 800. Actual: 400-405
// Target: 400. Actual: 384
// Target: 390. Actual: 377
// Target: 360. Actual: 355
// Target: 340. Actual: 338.4
// Target: 338. ACtual: 335.5
// Target: 335. Actual: 332.8
// Target: 330. Actual: 329.6
// The error on 330 is low enough that there are not sync issues.
const int max_imu_rate = 240; //Hz. Minimum: 129
const int imu_camera_ratio = max_imu_rate / max_camera_rate;

float previous_time_imu = 0;
// float previous_time_camera = 0;

// Start by triggering both.
int num_imu_ticks_since_last_photo = imu_camera_ratio - 1;
unsigned long current_time;

// Time between triggers based on rates (in micros)
// float camera_interval = 1000000.0 / max_camera_rate;
float imu_interval = 1000000.0 / max_imu_rate;

//Triggers each sensor
void sensorTriggering(int pin_number) {
  current_time = micros();
  digitalWrite(pin_number, HIGH);
  delayMicroseconds(100);
  digitalWrite(pin_number, LOW);

  if (pin_number == imu_pin) {
    ulong_msg1.data = current_time;
    imu_timestamps.publish(&ulong_msg1);
  }
  else if (pin_number == camera_pin) {
    ulong_msg2.data = current_time;
    cam_timestamps.publish(&ulong_msg2);
  }
}

// Triggers both sensors by sending a 100 microsecond long square wave
// to their digital pins
void triggerBothSensors() {
  current_time = micros();

  // PORTD represents the arduino digital ports 0-7 in bits 0-7.
  // Setting these two bits to true will thus set those pins to HIGH
  PORTD |= (1 << camera_pin) | (1 << imu_pin);

  delayMicroseconds(100);

  // Setting these two bits to false will thus set those pins to LOW
  PORTD ^= (1 << camera_pin) | (1 << imu_pin);
  
  ulong_msg1.data = current_time;
  ulong_msg2.data = current_time;
  imu_timestamps.publish(&ulong_msg1);
  cam_timestamps.publish(&ulong_msg2);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  nh.initNode();
//  nh.getHardware()->setBaud(115200);
  nh.advertise(imu_timestamps);
  nh.advertise(cam_timestamps);
  pinMode(camera_pin, OUTPUT);
  pinMode(imu_pin, OUTPUT);

  // delay several seconds to allow for setup time.
  int i = 0;
  while (i++ < 20) {
    nh.spinOnce();
    delay(1000);
  }
//  nh.spinOnce();
//  delay(30 * 1000);
  current_time = micros();
  previous_time_imu = current_time - imu_interval;
  // previous_time_camera = current_time - camera_interval;
}

void loop() {
  current_time = micros();

  if (current_time - previous_time_imu >= imu_interval) {
    num_imu_ticks_since_last_photo++;
    if (num_imu_ticks_since_last_photo == imu_camera_ratio) {
      triggerBothSensors();
      num_imu_ticks_since_last_photo = 0;
    } else {
      sensorTriggering(imu_pin);
    }
    previous_time_imu += imu_interval;
  }
  
  nh.spinOnce();
}
