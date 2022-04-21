#include <kelly.h>

void kellyInit()
{
  // TODO: add speedometer wires

  // #### BL Wheel ####
  pinMode(FWD_SW_BL, OUTPUT);
  pinMode(REV_SW_BL, OUTPUT);
  pinMode(BRK_SW_BL, OUTPUT);
  pinMode(THR_AN_BL, OUTPUT);
  pinMode(BRK_AN_BL, OUTPUT);

  // #### BR Wheel ####
  pinMode(FWD_SW_BR, OUTPUT);
  pinMode(REV_SW_BR, OUTPUT);
  pinMode(BRK_SW_BR, OUTPUT);
  pinMode(THR_AN_BR, OUTPUT);
  pinMode(BRK_AN_BR, OUTPUT);

  // #### FL Wheel ####
  pinMode(FWD_SW_FL, OUTPUT);
  pinMode(REV_SW_FL, OUTPUT);
  pinMode(BRK_SW_FL, OUTPUT);
  pinMode(THR_AN_FL, OUTPUT);
  pinMode(BRK_AN_FL, OUTPUT);

  // #### FR Wheel ####
  pinMode(FWD_SW_FR, OUTPUT);
  pinMode(REV_SW_FR, OUTPUT);
  pinMode(BRK_SW_FR, OUTPUT);
  pinMode(THR_AN_FR, OUTPUT);
  pinMode(BRK_AN_FR, OUTPUT);
}

void kellyBrake(unsigned int BRK_SW_PIN, unsigned int throttle_PWM_PIN, unsigned int brake_PWM_PIN, unsigned int BRK_MAX_PWM)
{
  digitalWrite(BRK_SW_PIN, HIGH);   // neutral, brake is on (High because the relay driver inverts the signal) // TODO: Send through I2C
  analogWrite(throttle_PWM_PIN, 1); // throttle zero makes a problem so never set it to zero
  analogWrite(brake_PWM_PIN, BRK_MAX_PWM);

  // uint8_t levels = BRK_MAX_PWM / 25; // 250/25 = 10 levels
  // uint8_t increment = BRK_MAX_PWM / levels;
  // for (size_t i = 1; i <= levels; i++)
  // {
  // uint8_t brake_value = increment * i;
  // analogWrite(brake_PWM_PIN, brake_value);
  // }
}

void kellyForward(unsigned int FWD_SW_PIN, unsigned int REV_SW_PIN, unsigned int BRK_SW_PIN, unsigned int throttle_PWM_PIN, unsigned int throttle_percentage, unsigned int FWD_MIN_PWM, unsigned int FWD_MAX_PWM)
{
  // throttle_precentage: commanded throttle by the app
  digitalWrite(FWD_SW_PIN, LOW);
  digitalWrite(REV_SW_PIN, HIGH);
  digitalWrite(BRK_SW_PIN, LOW); // brake is off
  analogWrite(throttle_PWM_PIN, map(throttle_percentage, 6, 100, FWD_MIN_PWM, FWD_MAX_PWM));
}

void kellyBackward(unsigned int FWD_SW_PIN, unsigned int REV_SW_PIN, unsigned int BRK_SW_PIN, unsigned int throttle_PWM_PIN, int throttle_percentage, unsigned int REV_MIN_PWM, unsigned int REV_MAX_PWM)
{
  digitalWrite(FWD_SW_PIN, HIGH);
  digitalWrite(REV_SW_PIN, LOW);
  digitalWrite(BRK_SW_PIN, LOW); // brake is off
  analogWrite(throttle_PWM_PIN, abs(map(throttle_percentage, -100, -6, REV_MAX_PWM, REV_MIN_PWM)));
}
