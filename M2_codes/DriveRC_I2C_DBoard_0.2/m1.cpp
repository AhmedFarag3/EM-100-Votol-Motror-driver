#include <Arduino.h>
#include <i2c_driver.h>
#include <i2c_driver_wire.h>
#include <kelly.h>
#include <steering.h>
void receiveEvent(int howMany);

// modes
// 0: idle
// 1: free mode
// 2: donut mode

// g_emergency_brake_msg
// 0: disabled (default)
// 1: enabled

// g_disc_brake_msg
// 0: disabled (default)
// 1: enabled

// g_throttle_direction
// 0: forward
// 1: reverse

// g_steering_direction
// 0: right
// 1: left

int g_emergency_brake_status = 0;
int g_current_mode = 0;
int g_prev_mode = 0;

int throttle_percentage = 0;
int steering_percentage = 0;

bool steering_state = false;

void setup()
{
  
  Can2.begin();
  Can2.setBaudRate(1000000);
  Can2.setMaxMB(16);
  Can2.enableFIFO();
  Can2.enableFIFOInterrupt();
  Can2.onReceive(canSniff);
  Can2.mailboxStatus();
  Can2.enhanceFilter(MB6);
  pack_cmd();
  
  Wire.begin(1);                // join i2c bus with address #8
  Wire.onReceive(receiveEvent); // register event
  Serial.begin(3000000);        // start serial for output
}

void loop()
{
  if (g_emergency_brake_status || g_current_mode == 0)
  {
    // ENABLE BRAKE
    // TODO: DISC BRAKE

    // Regenerative brake
    kellyBrake(rlybrake_bl, throttle_bl, brakeana_bl, maxbrake);
    kellyBrake(rlybrake_br, throttle_br, brakeana_br, maxbrake);
    kellyBrake(rlybrake_fl, throttle_fl, brakeana_fl, maxbrake);
    kellyBrake(rlybrake_fr, throttle_fr, brakeana_fr, maxbrake);
  }
  else if (!g_emergency_brake_status)
  {
    if (g_current_mode != g_prev_mode)
    {
      steering_state = LOW;
    }
    g_prev_mode = g_current_mode;

    if (g_current_mode == 1) // free mode
    {
      // THROTTLE
      if (10 >= throttle_percentage && -10 <= throttle_percentage)
      {
        // idle
        // ENABLE BRAKE
        // TODO: DISC BRAKE

        // Regenerative brake
        kellyBrake(rlybrake_bl, throttle_bl, brakeana_bl, maxbrake);
        kellyBrake(rlybrake_br, throttle_br, brakeana_br, maxbrake);
        kellyBrake(rlybrake_fl, throttle_fl, brakeana_fl, maxbrake);
        kellyBrake(rlybrake_fr, throttle_fr, brakeana_fr, maxbrake);
      }
      else if (10 < throttle_percentage && 100 >= throttle_percentage)
      {
        // forward
        kellyForward(rlyfw_bl, rlyrev_bl, rlybrake_bl, throttle_bl, throttle_percentage, minfwd, maxfwd);
        kellyForward(rlyfw_br, rlyrev_br, rlybrake_br, throttle_br, throttle_percentage, minfwd, maxfwd);
        kellyForward(rlyfw_fl, rlyrev_fl, rlybrake_fl, throttle_fl, throttle_percentage, minfwd, maxfwd);
        kellyForward(rlyfw_fr, rlyrev_fr, rlybrake_fr, throttle_fr, throttle_percentage, minfwd, maxfwd);
      }
      else if (-10 > throttle_percentage && -100 <= throttle_percentage)
      {
        // reverse
        kellyBackward(rlyfw_bl, rlyrev_bl, rlybrake_bl, throttle_bl, throttle_percentage, minfwd, maxfwd);
        kellyBackward(rlyfw_br, rlyrev_br, rlybrake_br, throttle_br, throttle_percentage, minfwd, maxfwd);
        kellyBackward(rlyfw_fl, rlyrev_fl, rlybrake_fl, throttle_fl, throttle_percentage, minfwd, maxfwd);
        kellyBackward(rlyfw_fr, rlyrev_fr, rlybrake_fr, throttle_fr, throttle_percentage, minfwd, maxfwd);
      }
      else
      {
        // ENABLE BRAKE
        // TODO: DISC BRAKE

        // Regenerative brake
        kellyBrake(rlybrake_bl, throttle_bl, brakeana_bl, maxbrake);
        kellyBrake(rlybrake_br, throttle_br, brakeana_br, maxbrake);
        kellyBrake(rlybrake_fl, throttle_fl, brakeana_fl, maxbrake);
        kellyBrake(rlybrake_fr, throttle_fr, brakeana_fr, maxbrake);
      }

      if (!steering_state)
      {
        // enable mode
        enableMode(msg_fr_1);
        enableMode(msg_fl_2);
        enableMode(msg_br_4);
        enableMode(msg_bl_3);
        steering_state = true;
      }

      // STEERING
      if (10 >= steering_percentage && -10 <= steering_percentage)
      {
        // idle
        // Set angles to 0
        pack_cmd(p_zero_fr, msg_fr_1);
        pack_cmd(p_zero_fl, msg_fl_2);
        pack_cmd(p_zero_br, msg_br_4);
        pack_cmd(p_zero_bl, msg_bl_3);
      }
      else if (10 < steering_percentage && 100 >= steering_percentage)
      {
        // right
        float current_steering_position_fr = map(steering_percentage, 11, 100, p_zero_fr, p_max_fr);
        float current_steering_position_fl = map(steering_percentage, 11, 100, p_zero_fl, p_max_fl);
        float current_steering_position_br = map(steering_percentage, 11, 100, p_zero_br, p_max_br);
        float current_steering_position_bl = map(steering_percentage, 11, 100, p_zero_bl, p_max_bl);

        pack_cmd(current_steering_position_fr, msg_fr_1);
        pack_cmd(current_steering_position_fl, msg_fl_2);
        pack_cmd(current_steering_position_br, msg_br_4);
        pack_cmd(current_steering_position_bl, msg_bl_3);
      }
      else if (-10 > steering_percentage && -100 <= steering_percentage)
      {
        // left
        float current_steering_position_fr = map(steering_percentage, -11, -100, p_zero_fr, p_min_fr);
        float current_steering_position_fl = map(steering_percentage, -11, -100, p_zero_fl, p_min_fl);
        float current_steering_position_br = map(steering_percentage, -11, -100, p_zero_br, p_min_br);
        float current_steering_position_bl = map(steering_percentage, -11, -100, p_zero_bl, p_min_bl);

        pack_cmd(current_steering_position_fr, msg_fr_1);
        pack_cmd(current_steering_position_fl, msg_fl_2);
        pack_cmd(current_steering_position_br, msg_br_4);
        pack_cmd(current_steering_position_bl, msg_bl_3);
      }
      else
      {
        // Set angles to 0
        pack_cmd(p_zero_fr, msg_fr_1);
        pack_cmd(p_zero_fl, msg_fl_2);
        pack_cmd(p_zero_br, msg_br_4);
        pack_cmd(p_zero_bl, msg_bl_3);
      }
    }
    else if (g_current_mode == 2) // spin mode
    {
      if (!steering_state)
      {
        // enable mode
        enableMode(msg_fr_1);
        enableMode(msg_fl_2);
        enableMode(msg_br_4);
        enableMode(msg_bl_3);
        // set steering angles
        pack_cmd(p_zero_fr, msg_fr_1);
        pack_cmd(p_zero_fl, msg_fl_2);
        pack_cmd(p_zero_br, msg_br_4);
        pack_cmd(p_zero_bl, msg_bl_3);

        // turn steering 45 degrees
        steering_state = true;
      }

      if (10 >= throttle_percentage && -10 <= throttle_percentage)
      {
        // idle
        // ENABLE BRAKE
        // TODO: DISC BRAKE

        // Regenerative brake
        kellyBrake(rlybrake_bl, throttle_bl, brakeana_bl, maxbrake);
        kellyBrake(rlybrake_br, throttle_br, brakeana_br, maxbrake);
        kellyBrake(rlybrake_fl, throttle_fl, brakeana_fl, maxbrake);
        kellyBrake(rlybrake_fr, throttle_fr, brakeana_fr, maxbrake);
      }
      else if (10 < throttle_percentage && 100 >= throttle_percentage)
      {
        // spin right
        kellyForward(rlyfw_bl, rlyrev_bl, rlybrake_bl, throttle_bl, throttle_percentage, minfwd, maxfwd);
        kellyForward(rlyfw_fl, rlyrev_fl, rlybrake_fl, throttle_fl, throttle_percentage, minfwd, maxfwd);
        kellyBackward(rlyfw_br, rlyrev_br, rlybrake_br, throttle_br, throttle_percentage, minfwd, maxfwd);
        kellyBackward(rlyfw_fr, rlyrev_fr, rlybrake_fr, throttle_fr, throttle_percentage, minfwd, maxfwd);
      }
      else if (-10 > throttle_percentage && -100 <= throttle_percentage)
      {
        // spin left
        kellyBackward(rlyfw_bl, rlyrev_bl, rlybrake_bl, throttle_bl, throttle_percentage, minfwd, maxfwd);
        kellyBackward(rlyfw_fl, rlyrev_fl, rlybrake_fl, throttle_fl, throttle_percentage, minfwd, maxfwd);
        kellyForward(rlyfw_br, rlyrev_br, rlybrake_br, throttle_br, throttle_percentage, minfwd, maxfwd);
        kellyForward(rlyfw_fr, rlyrev_fr, rlybrake_fr, throttle_fr, throttle_percentage, minfwd, maxfwd);
      }
      else
      {
        // ENABLE BRAKE
        // TODO: DISC BRAKE

        // Regenerative brake
        kellyBrake(rlybrake_bl, throttle_bl, brakeana_bl, maxbrake);
        kellyBrake(rlybrake_br, throttle_br, brakeana_br, maxbrake);
        kellyBrake(rlybrake_fl, throttle_fl, brakeana_fl, maxbrake);
        kellyBrake(rlybrake_fr, throttle_fr, brakeana_fr, maxbrake);
      }
    }
  }
}

void receiveEvent(int howMany)
{
  int receivedData = 0;
  while (Wire.available() >= 1)
  {                                            // loop through all but the last
    receivedData = receivedData + Wire.read(); // receive byte as integer
  }
  if (receivedData >= 1 && receivedData <= 201)
  {
    int Throttle = receivedData - 101; // -100 ~ 0 ~ 100
    throttle_percentage = Throttle;
    Serial.printf("Throttle: %d", Throttle);
  }
  else if (receivedData >= 300 && receivedData <= 500)
  {
    int Steering = receivedData - 400; // -100 ~ 0 ~ 100
    steering_percentage = Steering;
    Serial.printf("Steering: %d", Steering);
  }
  else if (receivedData == 250) // idle
  {
    g_current_mode = 0;
    Serial.print("IDLE");
  }
  else if (receivedData == 260) // free mode
  {
    g_current_mode = 1;
    Serial.print("FREE MODE");
  }
  else if (receivedData == 270) // spin
  {
    g_current_mode = 2;
    Serial.print("SPIN MODE");
  }
  else if (receivedData == 290)
  {
    g_emergency_brake_status = 1;
    Serial.print("EMERGENCY BRAKE");
  }
  else if (receivedData == 291)
  {
    g_emergency_brake_status = 0;
    Serial.print("NO EMERGENCY BRAKE");
  }
  else
  {
    g_current_mode = 0;
    throttle_percentage = 0;
    steering_percentage = 0;
    g_emergency_brake_status = 0;
    Serial.printf("NOT DEFINED: %d ", receivedData);
  }
  Serial.println();
}
