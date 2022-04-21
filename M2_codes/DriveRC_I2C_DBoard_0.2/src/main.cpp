/*
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief Burn on rc board
 * @version 0.1
 * @date 2022-02-14
 *
 * @copyright Copyright (c) 2022
 *
 */
// #include <Arduino.h>

#include <kelly.h>
#include <sensors_i2c.h>

// #include <Wire.h> // for master writer

#include <i2c_device.h> // for master/slave receiver
#include <i2c_driver_wire.h>

void SendNBytes(int data);
void receiveEvent0(int howMany0);
void receiveEvent2(int howMany2);
void requestEvent0();

// modes
// 0: idle
// 1: free mode
// 2: donut mode
// 3: spin mode

// g_emergency_brake_msg
// 0: disabled (default)
// 1: enabled

// g_throttle_direction
// 0: idle
// 1: forward
// -1: reverse

unsigned short g_emergency_brake_status = 0;
unsigned short g_current_mode = 0;
unsigned short g_prev_mode = 0;

short current_throttle_percentage = 0;
short previous_throttle_percentage = 0;
short g_throttle_direction = 0;

unsigned short disc_brake = 1;

const unsigned short drive_i2c_addr_wire0 = 1;
const unsigned short drive_i2c_addr_wire2 = 3;

bool state_changed = LOW;

// unsigned long previous_millis = 0;
// const unsigned long interval = 300;
#define DISC_BRK_PIN 11

void setup()
{
  kellyInit();

  pinMode(DISC_BRK_PIN, OUTPUT);

  Wire0_T4.begin(drive_i2c_addr_wire0); // join i2c bus as slave with address #1
  Wire0_T4.setClock(100000);
  Wire0_T4.onReceive(receiveEvent0); // register receive event

  // Join I2C_2 bus as slave receiver (use i2c_device... to receive as)
  Wire2_T4.begin(drive_i2c_addr_wire2);
  Wire2_T4.setClock(100000);
  Wire2_T4.onReceive(receiveEvent2);
  
  analogWrite(THR_AN_BL, 1);
  analogWrite(THR_AN_BR, 1);
  analogWrite(THR_AN_FL, 1);
  analogWrite(THR_AN_FR, 1);

  Serial.begin(3000000); // start serial for output
}
// uint64_t loop_counter;
void loop()
{
  // loop_counter++;
  // receiveEvent0(0);
  if ((g_emergency_brake_status || g_current_mode) == 0)
  {
    // ENABLE BRAKE
    // DISC BRAKE
    disc_brake = HIGH;
    digitalWrite(DISC_BRK_PIN, disc_brake);

    // Regenerative brake
    kellyBrake(BRK_SW_BL, THR_AN_BL, BRK_AN_BL, maxbrake);
    kellyBrake(BRK_SW_BR, THR_AN_BR, BRK_AN_BR, maxbrake);
    kellyBrake(BRK_SW_FL, THR_AN_FL, BRK_AN_FL, maxbrake);
    kellyBrake(BRK_SW_FR, THR_AN_FR, BRK_AN_FR, maxbrake);
  }

  else if (!g_emergency_brake_status)
  {
    if (g_current_mode == 1) // free mode
    {
      // Serial.printf("THROTTLE:  %d \n", current_throttle_percentage);
      // delay(50);
      if (10 >= current_throttle_percentage && -10 <= current_throttle_percentage) // Dead zone
      {
        // ENABLE BRAKE
        // DISC BRAKE
        disc_brake = HIGH;
        digitalWrite(DISC_BRK_PIN, disc_brake);

        // Regenerative brake
        kellyBrake(BRK_SW_BL, THR_AN_BL, BRK_AN_BL, maxbrake);
        kellyBrake(BRK_SW_BR, THR_AN_BR, BRK_AN_BR, maxbrake);
        kellyBrake(BRK_SW_FL, THR_AN_FL, BRK_AN_FL, maxbrake);
        kellyBrake(BRK_SW_FR, THR_AN_FR, BRK_AN_FR, maxbrake);

        g_throttle_direction = 0; // idle
      }
      else if (10 < current_throttle_percentage && 100 >= current_throttle_percentage) // forward
      {
        disc_brake = LOW;
        digitalWrite(DISC_BRK_PIN, disc_brake);

        kellyForward(FWD_SW_FR, REV_SW_FR, BRK_SW_FR, THR_AN_FR, current_throttle_percentage, minfwd, maxfwd);
        kellyForward(FWD_SW_FL, REV_SW_FL, BRK_SW_FL, THR_AN_FL, current_throttle_percentage, minfwd, maxfwd);
        kellyForward(FWD_SW_BR, REV_SW_BR, BRK_SW_BR, THR_AN_BR, current_throttle_percentage, minfwd, maxfwd);
        kellyForward(FWD_SW_BL, REV_SW_BL, BRK_SW_BL, THR_AN_BL, current_throttle_percentage, minfwd, maxfwd);

        g_throttle_direction = 1; // forward
        previous_throttle_percentage = current_throttle_percentage;
      }
      else if (-10 > current_throttle_percentage && -100 <= current_throttle_percentage) // reverse
      {
        disc_brake = LOW;
        digitalWrite(DISC_BRK_PIN, disc_brake);
        Serial.println("Reverse---------------------------");
        kellyBackward(FWD_SW_FR, REV_SW_FR, BRK_SW_FR, THR_AN_FR, current_throttle_percentage, minfwd, maxfwd);
        kellyBackward(FWD_SW_FL, REV_SW_FL, BRK_SW_FL, THR_AN_FL, current_throttle_percentage, minfwd, maxfwd);
        kellyBackward(FWD_SW_BR, REV_SW_BR, BRK_SW_BR, THR_AN_BR, current_throttle_percentage, minfwd, maxfwd);
        kellyBackward(FWD_SW_BL, REV_SW_BL, BRK_SW_BL, THR_AN_BL, current_throttle_percentage, minfwd, maxfwd);

        g_throttle_direction = -1; // reverse
        previous_throttle_percentage = current_throttle_percentage;
      }
      /* else
       {
         // ENABLE BRAKE
         // TODO: DISC BRAKE

         // Regenerative brake
         kellyBrake(BRK_SW_BL, THR_AN_BL, BRK_AN_BL, maxbrake);
         kellyBrake(BRK_SW_BR, THR_AN_BR, BRK_AN_BR, maxbrake);
         kellyBrake(BRK_SW_FL, THR_AN_FL, BRK_AN_FL, maxbrake);
         kellyBrake(BRK_SW_FR, THR_AN_FR, BRK_AN_FR, maxbrake);
       }*/
    }
    else if (g_current_mode == 2) // donut mode
    {
      if (10 >= current_throttle_percentage && -10 <= current_throttle_percentage) // throttle dead zone
      {
        // idle
        // ENABLE BRAKE
        // DISC BRAKE
        disc_brake = HIGH;
        digitalWrite(DISC_BRK_PIN, disc_brake);

        // Regenerative brake
        kellyBrake(BRK_SW_BL, THR_AN_BL, BRK_AN_BL, maxbrake);
        kellyBrake(BRK_SW_BR, THR_AN_BR, BRK_AN_BR, maxbrake);
        kellyBrake(BRK_SW_FL, THR_AN_FL, BRK_AN_FL, maxbrake);
        kellyBrake(BRK_SW_FR, THR_AN_FR, BRK_AN_FR, maxbrake);
      }
      else if (10 < current_throttle_percentage && 100 >= current_throttle_percentage) // throttle fwd
      {
        // release brake
        disc_brake = LOW;
        digitalWrite(DISC_BRK_PIN, disc_brake);

        // forward
        kellyForward(FWD_SW_FR, REV_SW_FR, BRK_SW_FR, THR_AN_FR, current_throttle_percentage, minfwd, maxfwd);
        kellyForward(FWD_SW_FL, REV_SW_FL, BRK_SW_FL, THR_AN_FL, current_throttle_percentage, minfwd, maxfwd);
        kellyForward(FWD_SW_BR, REV_SW_BR, BRK_SW_BR, THR_AN_BR, current_throttle_percentage, minfwd, maxfwd);
        kellyForward(FWD_SW_BL, REV_SW_BL, BRK_SW_BL, THR_AN_BL, current_throttle_percentage, minfwd, maxfwd);

        g_throttle_direction = 1; // forward
        previous_throttle_percentage = current_throttle_percentage;
      }
      else if (-10 > current_throttle_percentage && -100 <= current_throttle_percentage) // throttle rev
      {
        // release brake
        disc_brake = LOW;
        digitalWrite(DISC_BRK_PIN, disc_brake);

        // reverse
        kellyBackward(FWD_SW_FR, REV_SW_FR, BRK_SW_FR, THR_AN_FR, current_throttle_percentage, minfwd, maxfwd);
        kellyBackward(FWD_SW_FL, REV_SW_FL, BRK_SW_FL, THR_AN_FL, current_throttle_percentage, minfwd, maxfwd);
        kellyBackward(FWD_SW_BR, REV_SW_BR, BRK_SW_BR, THR_AN_BR, current_throttle_percentage, minfwd, maxfwd);
        kellyBackward(FWD_SW_BL, REV_SW_BL, BRK_SW_BL, THR_AN_BL, current_throttle_percentage, minfwd, maxfwd);

        g_throttle_direction = -1; // reverse
        previous_throttle_percentage = current_throttle_percentage;
      }
      /*else // any other value
      {
        // ENABLE BRAKE
        // TODO: DISC BRAKE

        // Regenerative brake
        kellyBrake(BRK_SW_BL, THR_AN_BL, BRK_AN_BL, maxbrake);
        kellyBrake(BRK_SW_BR, THR_AN_BR, BRK_AN_BR, maxbrake);
        kellyBrake(BRK_SW_FL, THR_AN_FL, BRK_AN_FL, maxbrake);
        kellyBrake(BRK_SW_FR, THR_AN_FR, BRK_AN_FR, maxbrake);
      }*/
    }
    else if (g_current_mode == 3) // spin mode
    {
      if (10 >= current_throttle_percentage && -10 <= current_throttle_percentage)
      {
        // idle
        // ENABLE BRAKE
        // DISC BRAKE
        // disc_brake = HIGH;
        // digitalWrite(DISC_BRK_PIN, disc_brake);

        // Regenerative brake
        // kellyBrake(BRK_SW_BL, THR_AN_BL, BRK_AN_BL, maxbrake);
        // kellyBrake(BRK_SW_BR, THR_AN_BR, BRK_AN_BR, maxbrake);
        // kellyBrake(BRK_SW_FL, THR_AN_FL, BRK_AN_FL, maxbrake);
        // kellyBrake(BRK_SW_FR, THR_AN_FR, BRK_AN_FR, maxbrake);
      }
      else if (10 < current_throttle_percentage && 100 >= current_throttle_percentage)
      {
        // release brake
        disc_brake = LOW;
        digitalWrite(DISC_BRK_PIN, disc_brake);

        // spin right
        // kellyForward(FWD_SW_BL, REV_SW_BL, BRK_SW_BL, THR_AN_BL, current_throttle_percentage, minfwd, maxfwd);
        // kellyForward(FWD_SW_FL, REV_SW_FL, BRK_SW_FL, THR_AN_FL, current_throttle_percentage, minfwd, maxfwd);
        // kellyBackward(FWD_SW_BR, REV_SW_BR, BRK_SW_BR, THR_AN_BR, current_throttle_percentage, minfwd, maxfwd);
        // kellyBackward(FWD_SW_FR, REV_SW_FR, BRK_SW_FR, THR_AN_FR, current_throttle_percentage, minfwd, maxfwd);

        g_throttle_direction = 1; // forward
        previous_throttle_percentage = current_throttle_percentage;
      }
      else if (-10 > current_throttle_percentage && -100 <= current_throttle_percentage)
      {
        // release brake
        disc_brake = LOW;
        digitalWrite(DISC_BRK_PIN, disc_brake);

        // spin left
        // kellyBackward(FWD_SW_BL, REV_SW_BL, BRK_SW_BL, THR_AN_BL, current_throttle_percentage, minfwd, maxfwd);
        // kellyBackward(FWD_SW_FL, REV_SW_FL, BRK_SW_FL, THR_AN_FL, current_throttle_percentage, minfwd, maxfwd);
        // kellyForward(FWD_SW_BR, REV_SW_BR, BRK_SW_BR, THR_AN_BR, current_throttle_percentage, minfwd, maxfwd);
        // kellyForward(FWD_SW_FR, REV_SW_FR, BRK_SW_FR, THR_AN_FR, current_throttle_percentage, minfwd, maxfwd);

        g_throttle_direction = -1; // reverse
        previous_throttle_percentage = current_throttle_percentage;
      }
      /*else
      {
        // ENABLE BRAKE
        // TODO: DISC BRAKE


        // Regenerative brake
        kellyBrake(BRK_SW_BL, THR_AN_BL, BRK_AN_BL, maxbrake);
        kellyBrake(BRK_SW_BR, THR_AN_BR, BRK_AN_BR, maxbrake);
        kellyBrake(BRK_SW_FL, THR_AN_FL, BRK_AN_FL, maxbrake);
        kellyBrake(BRK_SW_FR, THR_AN_FR, BRK_AN_FR, maxbrake);
      }*/
    }
  }
  delay(100);
}

void receiveEvent0(int howMany0)
{
  digitalWrite(13, HIGH);
  Serial.println("------------------------\nSTART OF RECEIVE EVENT 0\n------------------------");
  int receivedData = 0;

  while (Wire0_T4.available()) // if something is being transmitted over I2C bus 0
  {
    receivedData = receivedData + Wire0_T4.read(); // receive byte as integer
  }
  Serial.printf("Data: %d\n", receivedData);

  // check received command
  if (receivedData >= 1 && receivedData <= 201) // throttle value
  {
    int Throttle = receivedData - 101; // -100 ~ 0 ~ 100
    current_throttle_percentage = Throttle;
    Serial.printf("Throttle: %d\n", Throttle);
  }
  else if (receivedData == 250) // idle
  {
    g_current_mode = 0;
    Serial.println("IDLE");
  }
  else if (receivedData == 260) // free mode
  {
    g_current_mode = 1;
    Serial.println("FREE MODE");
  }
  else if (receivedData == 270) // donut
  {
    g_current_mode = 2;
    Serial.println("DONUT MODE");
  }
  else if (receivedData == 280) // spin
  {
    g_current_mode = 3;
    Serial.println("SPIN MODE");
  }
  else if (receivedData == 290) // disable emergency brake
  {
    g_emergency_brake_status = 0;
    Serial.println("NO EMERGENCY BRAKE");
  }
  else if (receivedData == 291) // enable emergency brake
  {
    g_emergency_brake_status = 1;
    Serial.println("EMERGENCY BRAKE");
  }
  else // garbage
  {
    g_current_mode = 0;
    current_throttle_percentage = 0;
    g_emergency_brake_status = 1;
    Serial.printf("NOT DEFINED: %d\n", receivedData);
  }

  digitalWrite(13, LOW);
  Serial.println("----------------------\nEND OF RECEIVE EVENT 0\n----------------------\n");
}

void receiveEvent2(int howMany2)
{
  // if (  Wire2_T4.read() == 291) // enable emergency brake
  // {
  //   // g_emergency_brake_status = 1;
  //   Serial.println("EMERGENCY BRAKE");
  // }
  // else
  // {

  for (int i = 0; i < howMany2 / 2; i++)
  {
    // fill sensors data array (received data)
    x = Wire2_T4.read();
    j++;
    if (j == 1)
    {
      y = (x << 8) | Wire2_T4.read();
      sensors_data_received[i] = y;
      // Serial.print(y, DEC);
      // Serial.print(' ');
      j = 0;
    }
  }
  // fill array that will be sent in the receive event for near real time transmission
  if (i2c_garmin1_flag == sensors_data_received[0])
  {
    Garmin1 = sensors_data_received[1];
    // sensors_data_to_be_sent[0] = i2c_garmin1_flag;
    // sensors_data_to_be_sent[1] = Garmin1;
    Serial.printf("Garmin 1: %0.2f m\n", ((float)Garmin1) / 100.0);
  }
  else if (i2c_garmin2_flag == sensors_data_received[0])
  {
    Garmin2 = sensors_data_received[1];
    // sensors_data_to_be_sent[0] = i2c_garmin2_flag;
    // sensors_data_to_be_sent[1] = Garmin2;
    Serial.printf("Garmin 2: %0.2f m\n", ((float)Garmin2) / 100.0);
  }
  else if (i2c_garmin3_flag == sensors_data_received[0])
  {
    Garmin3 = sensors_data_received[1];
    // sensors_data_to_be_sent[0] = i2c_garmin3_flag;
    // sensors_data_to_be_sent[1] = Garmin3;
    Serial.printf("Garmin 3: %0.2f m\n", ((float)Garmin3) / 100.0);
  }
  else if (i2c_garmin4_flag == sensors_data_received[0])
  {
    Garmin4 = sensors_data_received[1];
    // sensors_data_to_be_sent[0] = i2c_garmin4_flag;
    // sensors_data_to_be_sent[1] = Garmin4;
    Serial.printf("Garmin 4: %0.2f m\n", ((float)Garmin4) / 100.0);
  }

  if (i2c_spot1_flag == sensors_data_received[2])
  {
    Radar1 = sensors_data_received[3];
    // sensors_data_to_be_sent[2] = i2c_spot1_flag;
    // sensors_data_to_be_sent[3] = Radar1;
    Serial.printf("Radar 1: %d\n", Radar1);
  }
  else if (i2c_spot2_flag == sensors_data_received[2])
  {
    Radar2 = sensors_data_received[3];
    // sensors_data_to_be_sent[2] = i2c_spot2_flag;
    // sensors_data_to_be_sent[3] = Radar2;
    Serial.printf("Radar 2: %d\n", Radar2);
  }

  Serial.print("Array = ");
  for (int i = 0; i < 4; i++)
  {
    Serial.printf("%d, ", sensors_data_received[i]);
  }
  Serial.println();
  // }
}

void SendNBytes(int data)
{
  digitalWrite(13, HIGH);
  double numDivision = data / 255.0; // divided by 255 because 1 byte = 255
  int remainder = data % 255;
  int numbOfIteration = 0;

  if (numDivision > int(numDivision))
    numbOfIteration = int(numDivision) + 1;
  else
    numbOfIteration = int(numDivision);

  // Serial.println("Data " + String(data));                        // print the character
  // Serial.println("numDivision: " + String(numDivision));         // print the character
  // Serial.println("remainder: " + String(remainder));             // print the character
  // Serial.println("numbOfIteration: " + String(numbOfIteration)); // print the character
  // Serial.println("--------------------");                        // print the character

  // Wire0_T4.beginTransmission(ADDR_GARMIN);

  for (int x = 0; x < numbOfIteration; x++)
  {
    if (remainder != 0)
    {
      if (x == numbOfIteration - 1)
      {
        Wire0_T4.write(remainder);
        //        Serial.println("send: " + String(remainder));         // print the character
      }
      else
      {
        Wire0_T4.write(255);
        //        Serial.println("send: " + String(255));         // print the character
      }
    }
    else
    {
      Wire0_T4.write(255);
    }
  }
  digitalWrite(13, LOW);
  // Wire0_T4.endTransmission();
}

// uint8_t i = 0;
// void requestEvent0()
// {
//   // send sensor 1
//   if (i == 0)
//   {
//     Wire0_T4.write(100);
//     delay(10);
//     // send 100 to assign that the next value is for Radar 1
//   }
//   else if (i == 1)
//   {
//     Wire0_T4.write(disc_brake + 1); // send 0 = 1, or 1 = 2
//     delay(10);
//   }

//   ++i == 2 ? i = 0 : i = i; // ON EACH REQUEST, A DATA IS SENT
// }
