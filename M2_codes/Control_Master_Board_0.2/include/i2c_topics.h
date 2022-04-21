#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>

#include <Arduino.h>
#include <Wire.h> // for master writer

#include <i2c_device.h> // for master/slave receiver
#include <i2c_driver_wire.h>

String freeModeStr = "freeMode";
String donutModeStr = "donutMode";
String spinModeStr = "spinMode";
String idleStr = "brake";
String g_mode_msg = "";
int g_emergency_brake_msg = 0; // emergency brake is off by default/

// g_direction_msg
// 0: forward
// 1: reverse
int g_direction_msg = 0; // direction is forward by default

const unsigned short drive_i2c_addr_wire0 = 1;
const unsigned short steer_i2c_addr = 2;
const unsigned short drive_i2c_addr_wire2 = 3;
const unsigned short brake_i2c_addr = 4;
const unsigned short lights_i2c_addr = 5;

unsigned int disc_brake;
// bool BrakeStatusRead = LOW;
// bool BrakePWMRead = LOW;

int Garmin1 = 0;
bool Garmin1_Read = LOW;

int Garmin2 = 0;
bool Garmin2_Read = LOW;

int Garmin3 = 0;
bool Garmin3_Read = LOW;

int Garmin4 = 0;
bool Garmin4_Read = LOW;

int Radar1 = 0;
bool Radar1_Read = LOW;

int Radar2 = 0;
bool Radar2_Read = LOW;

ros::NodeHandle nh;

#define i2c_brakeMode_inc 250
#define i2c_freeMode_inc 260
#define i2c_donutMode_inc 270
#define i2c_spinMode_inc 280
#define i2c_throttle_inc 101
#define i2c_steering_inc 400
#define i2c_emergency_inc 290
#define i2c_lights_full_inc 210
#define i2c_lights_head_inc 215
#define i2c_siren1_inc 220
#define i2c_siren2_inc 225
#define i2c_siren3_inc 230
#define i2c_siren_light_inc 235

#define i2c_garmin1_flag 1500
#define i2c_garmin2_flag 1510
#define i2c_garmin3_flag 1520
#define i2c_garmin4_flag 1530

#define i2c_spot1_flag 610
#define i2c_spot2_flag 620

#define FWD_MAX_PWM 160
#define FWD_MIN_PWM 60
#define REV_MAX_PWM 160
#define REV_MIN_PWM 60
#define BRK_MAX_PWM 250
#define BRK_MIN_PWM 20

void sendNBytesFromMaster(const unsigned short i2c_addr, int data)
{
  double numDivision = data / 255.0; // divided by 255 because 1 byte
  int remainder = data % 255;
  int numbOfIteration = 0;

  if (numDivision > int(numDivision))
    numbOfIteration = int(numDivision) + 1;
  else
    numbOfIteration = int(numDivision);

  Wire.beginTransmission(i2c_addr); // transmit to device #8
  for (int i = 0; i < numbOfIteration; i++)
  {
    if (remainder != 0)
    {
      if (i == numbOfIteration - 1)
      {
        Wire.write(remainder);
        // Serial.println("send: " + String(remainder));         // print the character
      }
      else
      {
        Wire.write(255);
        // Serial.println("send: " + String(255));         // print the character
      }
      // Serial.println(i);
    }
    else
    {

      Wire.write(255);
    }
  }
  Wire.endTransmission(); // stop transmitting
}

void i2c_drive_mode(const unsigned short i2c_addr1, const unsigned short i2c_addr2, String drive_mode)
{
  if (drive_mode == freeModeStr.trim())
  {
    sendNBytesFromMaster(i2c_addr1, i2c_freeMode_inc);
    sendNBytesFromMaster(i2c_addr2, i2c_freeMode_inc);

    // nh.loginfo((String("\nI2C Mode") + String(i2c_freeMode_inc).c_str()).c_str());
    // nh.loginfo((String("FREE\n")).c_str());
  }
  else if (drive_mode == donutModeStr.trim())
  {
    sendNBytesFromMaster(i2c_addr1, i2c_donutMode_inc);
    sendNBytesFromMaster(i2c_addr2, i2c_donutMode_inc);

    // nh.loginfo((String("\nI2C Mode") + String(i2c_donutMode_inc).c_str()).c_str());
    // nh.loginfo((String("DONUT\n")).c_str());
  }
  else if (drive_mode == spinModeStr.trim())
  {
    sendNBytesFromMaster(i2c_addr1, i2c_spinMode_inc);
    sendNBytesFromMaster(i2c_addr2, i2c_spinMode_inc);

    // nh.loginfo((String("\nI2C Mode") + String(i2c_spinMode_inc).c_str()).c_str());
    // nh.loginfo((String("SPIN\n")).c_str());
  }
  else if (drive_mode == idleStr.trim())
  {
    sendNBytesFromMaster(i2c_addr1, i2c_brakeMode_inc);

    // nh.loginfo((String("\nI2C Mode") + String(i2c_brakeMode_inc).c_str()).c_str());
    // nh.loginfo((String("IDLE\n")).c_str());
  }
  else
  {
    sendNBytesFromMaster(i2c_addr1, BRK_MAX_PWM);

    // nh.loginfo((String("\n[GARBAGE_DRIVE_MODE] Mode") + String(BRK_MAX_PWM).c_str()).c_str());
    // nh.loginfo((String("BRAKE\n")).c_str());
  }
}

void i2c_throttle(const unsigned short i2c_addr1, const unsigned short i2c_addr2, unsigned int throttle_percentage, String drive_mode)
{
  int sendThrottle = 0;

  if (drive_mode == spinModeStr.trim())
  {
    sendThrottle = throttle_percentage + i2c_steering_inc;
    sendNBytesFromMaster(i2c_addr2, sendThrottle);
  }
  else
  {
    sendThrottle = throttle_percentage + i2c_throttle_inc;
    sendNBytesFromMaster(i2c_addr1, sendThrottle);
  }
  nh.loginfo((String("i2c_throttle_percentage") + String(sendThrottle).c_str()).c_str());
}

void i2c_direction(const unsigned short i2c_addr, int direction)
{
  sendNBytesFromMaster(i2c_addr, direction);
}

void i2c_steering(const unsigned short i2c_addr, unsigned int steering_percentage)
{
  int sendSteering = steering_percentage + i2c_steering_inc;
  sendNBytesFromMaster(i2c_addr, sendSteering);

  // nh.loginfo((String("i2c_steering_percentage") + String(sendSteering).c_str()).c_str());
}

void i2c_emergency_brake(const unsigned short i2c_addr, int emergency_brake_state)
{
  int sendEmBrake = emergency_brake_state + i2c_emergency_inc;
  sendNBytesFromMaster(i2c_addr, sendEmBrake);

  // nh.loginfo((String("i2c_emergency_brake_status") + String(sendEmBrake).c_str()).c_str());
}

void i2c_lights_full(const unsigned short i2c_addr1, const unsigned short i2c_addr2, int lights_full)
{
  int sendLightsFull = lights_full + i2c_lights_full_inc;
  sendNBytesFromMaster(i2c_addr1, sendLightsFull);
  sendNBytesFromMaster(i2c_addr2, sendLightsFull);
  // nh.loginfo((String("i2c_lights_full") + String(lights_full).c_str()).c_str());
}

void i2c_lights_head(const unsigned short i2c_addr1, const unsigned short i2c_addr2, int lights_head)
{
  int sendLightsHead = lights_head + i2c_lights_head_inc;
  sendNBytesFromMaster(i2c_addr1, sendLightsHead);
  sendNBytesFromMaster(i2c_addr2, sendLightsHead);
  // nh.loginfo((String("i2c_lights_head") + String(lights_head).c_str()).c_str());
}

void i2c_siren1(const unsigned short i2c_addr1, const unsigned short i2c_addr2, int siren1)
{
  int sendSiren1 = siren1 + i2c_siren1_inc;
  sendNBytesFromMaster(i2c_addr1, sendSiren1);
  sendNBytesFromMaster(i2c_addr2, sendSiren1);
  // nh.loginfo((String("i2c_siren1") + String(siren1).c_str()).c_str());
}

void i2c_siren2(const unsigned short i2c_addr1, const unsigned short i2c_addr2, int siren2)
{
  int sendSiren2 = siren2 + i2c_siren2_inc;
  sendNBytesFromMaster(i2c_addr1, sendSiren2);
  sendNBytesFromMaster(i2c_addr2, sendSiren2);
  // nh.loginfo((String("i2c_siren2") + String(siren2).c_str()).c_str());
}

void i2c_siren3(const unsigned short i2c_addr1, const unsigned short i2c_addr2, int siren3)
{
  int sendSiren3 = siren3 + i2c_siren3_inc;
  sendNBytesFromMaster(i2c_addr1, sendSiren3);
  sendNBytesFromMaster(i2c_addr2, sendSiren3);
  // nh.loginfo((String("i2c_siren3") + String(siren3).c_str()).c_str());
}

void i2c_siren_light(const unsigned short i2c_addr1, const unsigned short i2c_addr2, int siren_light)
{
  int sendSirenLight = siren_light + i2c_siren_light_inc;
  sendNBytesFromMaster(i2c_addr1, sendSirenLight);
  sendNBytesFromMaster(i2c_addr2, sendSirenLight);
  // nh.loginfo((String("i2c_siren_light") + String(siren_light).c_str()).c_str());
}

/*void i2c_disc_brake(const unsigned short i2c_read_addr, const unsigned short i2c_write_addr, unsigned int &disc_brake_status, unsigned int &disc_brake_value)
{
  Wire.requestFrom(i2c_read_addr, 5); // request 5 bytes from slave device #10

  int i2c_msg = 0;
  while (Wire.available())
  {                         // slave may send less than requested
    i2c_msg += Wire.read(); // receive a byte as character
  }
  Serial.printf("msg: %d \n", i2c_msg);

  if (BrakeStatusRead)
  {
    disc_brake_status = i2c_msg;
    sendNBytesFromMaster(i2c_write_addr, disc_brake_status);
    BrakeStatusRead = LOW;
    Serial.printf("[CONTROL] Brake Status: %d \n", disc_brake_status);
    delay(5);
  }
  // else if (BrakePWMRead)
  // {
  //   disc_brake_value = i2c_msg;
  //   sendNBytesFromMaster(i2c_write_addr, disc_brake_value);
  //   BrakePWMRead = LOW;
  //   Serial.printf("[CONTROL] Brake PWM Value: %d \n", disc_brake_value);
  //   delay(5);
  // }

  if (i2c_msg == 100) // 115 = 's'
  {
    sendNBytesFromMaster(i2c_write_addr, 100);
    BrakeStatusRead = HIGH;
    // Serial.println("STATUS");
    // delay(5);
  }
  // else if ((char)i2c_msg == 'b') // 98 = 'b'
  // {
  //   sendNBytesFromMaster(i2c_write_addr, 98);
  //   BrakePWMRead = HIGH;
  //   // Serial.println("PWM");
  //   // delay(5);
  // }
}
*/
// ######################## SENSROS ########################

// void i2c_garmin(const int i2c_addr, unsigned int sensor_msg, unsigned int which_sensor)
// {
//   /*
//    * @i2c_addr: I2C Address of the slave board to which data is to be transmitted
//    *
//    * @sensor_msg: Data message from sensor
//    *
//    * @which_sensor: Specifiy which sensor has sent this frame of data
//    *
//    */

//   switch (which_sensor)
//   {
//   case 1:
//     sendNBytes(i2c_addr, i2c_garmin1_flag);
//     break;
//   case 2:
//     sendNBytes(i2c_addr, i2c_garmin2_flag);
//     break;
//   case 3:
//     sendNBytes(i2c_addr, i2c_garmin3_flag);
//     break;
//   case 4:
//     sendNBytes(i2c_addr, i2c_garmin4_flag);
//     break;

//   default:
//     break;
//   }

//   sendNBytes(i2c_addr, sensor_msg);
//   nh.loginfo((String("i2c_garmin_") + String(which_sensor).c_str() + String(" = ") + String(sensor_msg).c_str()).c_str());
// }

// void i2c_spot_radar(const int i2c_addr, unsigned int sensor_msg, unsigned int which_sensor)
// {
//   /*
//    * @i2c_addr: I2C Address of the slave board to which data is to be transmitted
//    *
//    * @sensor_msg: Data message from sensor
//    *
//    * @which_sensor: Specifiy which sensor has sent this frame of data
//    *
//    */

//   switch (which_sensor)
//   {
//   case 1:
//     sendNBytes(i2c_addr, i2c_spot1_flag);
//     break;
//   case 2:
//     sendNBytes(i2c_addr, i2c_spot2_flag);
//     break;

//   default:
//     break;
//   }

//   sendNBytes(i2c_addr, sensor_msg);
//   nh.loginfo((String("i2c_spot_radar_") + String(which_sensor).c_str() + String(" = ") + String(sensor_msg).c_str()).c_str());
// }
