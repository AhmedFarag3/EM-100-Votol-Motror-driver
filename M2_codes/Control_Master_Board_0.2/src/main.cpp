/*
 * MASTER CONTROL TEENSY CODE
 * ------------------
 * Subscribe to topics from ROS network
 * and send them through I2C
 *
 */

#include <i2c_topics.h>

void receiveEvent(int howMany);

// ros::NodeHandle nh;

void DriveModeCb(const std_msgs::String &mode_msg)
{
  // String msg = String(mode_msg.data).trim();
  g_mode_msg = String(mode_msg.data).trim();
  i2c_drive_mode(drive_i2c_addr_wire0, steer_i2c_addr, g_mode_msg);

  // Debugging
  nh.loginfo((String("Drive Mode = ") + (g_mode_msg.c_str())).c_str());
}

void EmergencyBrakeCb(const std_msgs::Int16 &em_brake_msg)
{
  g_emergency_brake_msg = int(em_brake_msg.data);
  // nh.loginfo((String("g_mode_msg = ") + (g_mode_msg.c_str())).c_str());
  i2c_emergency_brake(drive_i2c_addr_wire0, g_emergency_brake_msg);

  // Debugging
  nh.loginfo((String("Emergency Brake State = ") + String(g_emergency_brake_msg).c_str()).c_str());
}

void ThrottleCb(const std_msgs::Int16 &throttle_msg)
{
  i2c_throttle(drive_i2c_addr_wire0, steer_i2c_addr, int(throttle_msg.data), g_mode_msg);
  // if (int(throttle_msg.data) > 50)
  //   digitalWrite(13, HIGH);
  // else
  //   digitalWrite(13, LOW);

  // Debugging
  nh.loginfo((String("Throttle = ") + String(throttle_msg.data).c_str()).c_str());
}

void SteeringCb(const std_msgs::Int16 &steering_msg)
{
  i2c_steering(steer_i2c_addr, int(steering_msg.data));

  // Debugging
  nh.loginfo((String("Steering = ") + String(steering_msg.data).c_str()).c_str());
}
////// ####################################

void FullLightCb(const std_msgs::Int16 &light_full_msg)
{
  i2c_lights_full(steer_i2c_addr, lights_i2c_addr, int(light_full_msg.data));

  // Debugging
  nh.loginfo((String("Full Lights = ") + String(light_full_msg.data).c_str()).c_str());
}

void HeadLightCb(const std_msgs::Int16 &light_head_msg)
{
  i2c_lights_head(steer_i2c_addr, lights_i2c_addr, int(light_head_msg.data));

  // Debugging
  nh.loginfo((String("Head Lights = ") + String(light_head_msg.data).c_str()).c_str());
}

void Siren1Cb(const std_msgs::Int16 &siren1_msg)
{
  i2c_siren1(steer_i2c_addr, lights_i2c_addr, int(siren1_msg.data));

  // Debugging
  nh.loginfo((String("Siren 1 = ") + String(siren1_msg.data).c_str()).c_str());
}

void Siren2Cb(const std_msgs::Int16 &siren2_msg)
{
  i2c_siren2(steer_i2c_addr, lights_i2c_addr, int(siren2_msg.data));

  // Debugging
  nh.loginfo((String("Siren 2 = ") + String(siren2_msg.data).c_str()).c_str());
}

void Siren3Cb(const std_msgs::Int16 &siren3_msg)
{
  i2c_siren3(steer_i2c_addr, lights_i2c_addr, int(siren3_msg.data));

  // Debugging
  nh.loginfo((String("Siren 3 = ") + String(siren3_msg.data).c_str()).c_str());
}

void SirenLightCb(const std_msgs::Int16 &siren_light_msg)
{
  i2c_siren_light(steer_i2c_addr, lights_i2c_addr, int(siren_light_msg.data));

  // Debugging
  nh.loginfo((String("Siren Light = ") + String(siren_light_msg.data).c_str()).c_str());
}

// std_msgs::Int16 Vehicle_Battery;
// std_msgs::Int16 Vehicle_throttle;

// ros::Publisher chatter_batt("Vehicle_Battery", &Vehicle_Battery);
// ros::Publisher chatter_acc("Vehicle_throttle", &Vehicle_throttle);

ros::Subscriber<std_msgs::String> driveModeSubscriber("ros_drive_mode", &DriveModeCb);
ros::Subscriber<std_msgs::Int16> steeringSubscriber("ros_steering", &SteeringCb);
ros::Subscriber<std_msgs::Int16> throttleSubscriber("ros_throttle", &ThrottleCb);
ros::Subscriber<std_msgs::Int16> emergencyBrakeSubscriber("ros_emergency_brake", &EmergencyBrakeCb);
ros::Subscriber<std_msgs::Int16> fullLightSubscriber("ros_lights_full", &FullLightCb);
ros::Subscriber<std_msgs::Int16> headLightSubscriber("ros_lights_head", &HeadLightCb);
ros::Subscriber<std_msgs::Int16> siren1Subscriber("ros_siren1", &Siren1Cb);
ros::Subscriber<std_msgs::Int16> siren2Subscriber("ros_siren2", &Siren2Cb);
ros::Subscriber<std_msgs::Int16> siren3Subscriber("ros_siren3", &Siren3Cb);
ros::Subscriber<std_msgs::Int16> sirenLightSubscriber("ros_siren_light", &SirenLightCb);

// byte x;
// int y;
// int j;
// int sensors_data_received[4];
// int sensors_data_to_be_sent[4];
// int transmit_result;

void setup()
{
  pinMode(11, INPUT);
  pinMode(13, OUTPUT);
  nh.initNode(); // Initialize node handle

  nh.subscribe(driveModeSubscriber);
  nh.subscribe(steeringSubscriber);
  nh.subscribe(throttleSubscriber);
  nh.subscribe(emergencyBrakeSubscriber);
  nh.subscribe(fullLightSubscriber);
  nh.subscribe(headLightSubscriber);
  nh.subscribe(siren1Subscriber);
  nh.subscribe(siren2Subscriber);
  nh.subscribe(siren3Subscriber);
  nh.subscribe(sirenLightSubscriber);

  // nh.advertise(chatter_batt);
  // nh.advertise(chatter_acc);

  Serial.begin(115000);

  // Join I2C_0 bus as master writer (only Wire.h works with master writer)
  Wire.begin();

  // Join I2C_2 bus as slave receiver (use i2c_device... to receive as)
  // Wire2_T4.begin(6);
  // Wire2_T4.setClock(1000000);
  // Wire2_T4.onReceive(receiveEvent);
}

void loop()
{
  // Vehicle_Battery.data = random(0, 100);
  // Vehicle_throttle.data = random(0, 10);
  // disc_brake = digitalRead(11);
  // nh.loginfo((String("Disc Brake = ") + (String(disc_brake).c_str())).c_str());
  // delay(50);
  // sendNBytesFromMaster(steer_i2c_addr,50);
  // chatter_batt.publish(&Vehicle_Battery);
  // chatter_acc.publish(&Vehicle_throttle);
  // Serial.print(Wire.available());
  nh.spinOnce();
  // delayMicroseconds(10);
  delay(1);
}