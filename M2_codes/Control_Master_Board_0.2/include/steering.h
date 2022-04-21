/// CAN Command Packet Structure ///
  /// 16 bit position command, between -4*pi and 4*pi
  /// 12 bit kp, between 0 and 500 N-m/rad
  /// 12 bit kd, between 0 and 100 N-m*s/rad
  /// 12 bit feed forward torque, between -18 and 18 N-m
  /// CAN Packet is 8 8-bit words
  /// Formatted as follows. For each quantity, bit 0 is LSB
  /// 0: [position[15-8]]
  /// 1: [position[7-0]]
  /// 2: [velocity[11-4]]
  /// 3: [velocity[0-3], kp[11-8]]
  /// 4: [kp[7-0]]
  /// 5: [kd[11-4]]
  /// 6: [Kd[3-0], torque[11-8]]
  /// 7: [torque[7-0]]

#include <FlexCAN_T4.h>

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

//#define sw1 35
//#define sw2 36

//Value Limits
#define P_MIN -95.5f
#define P_MAX 95.5f
#define V_MIN -25.64f
#define V_MAX 25.64f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
#define POS_STEP 0.2f

//Set values
// Front right (1)
float p_in_fr = -10.0f;     //Initial position
float p_zero_fr = p_in_fr;  //Initial Zero position
float p_min_fr = -16.0f;    //Minimum position
float p_max_fr = -3.0f;     //Max Position

// Front left (2)
float p_in_fl = -0.5f;      //Initial position
float p_zero_fl = p_in_fl;  //Initial Zero position
float p_min_fl = -8.0f;     //Minimum position
float p_max_fl = 7.0f;      //Max Position

// Back left (3)
float p_in_bl = -7.0f;      //Initial position
float p_zero_bl = p_in_bl;  //Initial Zero position
float p_min_bl = 0.0f;      //Minimum position
float p_max_bl = -13.0f;    //Max Position

// Back right (4)
float p_in_br = -15.5f;     //Initial position
float p_zero_br = p_in_br;  //Initial Zero position
float p_min_br = -20.0f;    //Minimum position
float p_max_br = -10.0f;    //Max Position

float v_in = 0.0f;
float kp_in = 100.0f;   // Gain
float kd_in = 1.0f;     // Derivative
float t_in = 12.0f;

//measured values
// float p_out = 0.0f;
// float v_out = 0.0f;
// float t_out = 0.0f;

CAN_message_t msg_fr_1 = {0x1};
CAN_message_t msg_fl_2 = {0x2};
CAN_message_t msg_bl_3 = {0x3};
CAN_message_t msg_br_4 = {0x4};

//toggle
int testtoggle = 0;
int w = 0;
unsigned int init_pos = 0;

unsigned int float_to_uint(float x, float x_min, float x_max, int bit)
{ /// converts a float to an unsigned int, given range and number of bits ///
  //Serial.println("conv2");
  //Serial.println(x);
  //delay(20);
  float span = x_max - x_min;
  float offset = x_min;
  //Serial.println(offset);
  unsigned int pgg = 0;
  if (bit == 12)
  {
    pgg = (x - offset) * 4095 / span;
  }
  if (bit == 16)
  {
    pgg = (x - offset) * 65535 / span;
  }
  return pgg;
}

void enableMode(CAN_message_t msg)
{
  Serial.println("Enter Motor Mode");
//   CAN_message_t msg;
//   msg.id = 0x1;
  msg.buf[0] = 0xFF;
  msg.buf[1] = 0xFF;
  msg.buf[2] = 0xFF;
  msg.buf[3] = 0xFF;
  msg.buf[4] = 0xFF;
  msg.buf[5] = 0xFF;
  msg.buf[6] = 0xFF;
  msg.buf[7] = 0xFC;
//   Can2.writ e(msg);
}

void Exit(CAN_message_t msg)
{
  Serial.println("Exit");
//   CAN_message_t msg;
//   msg.id = 0x1;
  msg.buf[0] = 0xFF;
  msg.buf[1] = 0xFF;
  msg.buf[2] = 0xFF;
  msg.buf[3] = 0xFF;
  msg.buf[4] = 0xFF;
  msg.buf[5] = 0xFF;
  msg.buf[6] = 0xFF;
  msg.buf[7] = 0xFD;
//   Can2.write(msg);
}

void canSniff(const CAN_message_t &msg) {
  Serial.print("MB ");
  Serial.print(msg.mb);
  Serial.print(" OVERRUN: ");
  Serial.print(msg.flags.overrun);
  Serial.print(" LEN: ");
  Serial.print(msg.len);
  Serial.print(" EXT: ");
  Serial.print(msg.flags.extended);
  Serial.print(" TS: ");
  Serial.print(msg.timestamp);
  Serial.print(" ID: ");
  Serial.print(msg.id, HEX);
  Serial.print(" Buffer: ");
  for ( uint8_t i = 0; i < msg.len; i++ ) {
    Serial.print(msg.buf[i], HEX); Serial.print(" ");
  }
  Serial.println();
}

void pack_cmd(float p_in, CAN_message_t msg)
{
  //  byte buf[8];
//   CAN_message_t msg;
//   msg.id = 0x1;

  ///limit data to be within bounds ///
  float p_des = constrain(p_in, P_MIN, P_MAX);
  float v_des = constrain(v_in, V_MIN, V_MAX);
  float kp = constrain(kp_in, KP_MIN, KP_MAX);
  float kd = constrain(kd_in, KD_MIN, KD_MAX);
  float t_ff = constrain(t_in, T_MIN, T_MAX);
  //convert floats to unsigned ints ///
  unsigned int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  Serial.println(p_in);

  Serial.println(p_des);

  Serial.println(p_int);
  //delay(10);
  unsigned int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  //Serial.println(v_int, HEX);
  //delay(10);
  unsigned int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  //Serial.println(kp_int, HEX);
  //delay(10);
  unsigned int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  //Serial.println(kd_int, HEX);
  //delay(10);
  unsigned int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  //Serial.println(t_int, HEX);
  //delay(10);
  //pack ints into the can buffer ///

  msg.buf[0] = p_int >> 8;
  msg.buf[1] = p_int & 0xFF;
  msg.buf[2] = v_int >> 4;
  msg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  msg.buf[4] = kp_int & 0xFF;
  msg.buf[5] = kd_int >> 4;
  msg.buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  msg.buf[7] = t_int & 0xFF;

  Can2.write(msg);

  delay(10);
}





/*
  void unpack_reply()
  {
  ///CAN Reply Packet Structure ///
  /// 16 bit position, between -4*pi and 4*pi
  /// 12 bit velocity, between -30 and +30 rad/s
  /// 12 bit current, between -40 and +40;
  /// CAN Packet is 5 8-bit words
  /// Formatted as follows. For each quantity, bit 0 is LSB
  /// 0: [position[15-8]]
  /// 1: [postion[7-0]
  /// 2: [velocity[11-4]]
  /// 3: [velocity[3-0], current[11-8]]
  /// 4: [current[7-0]]
  Serial.println("unpacking");

  byte len = 0;
  byte buf[8];
  CAN.readMsgBuf(&len, buf);

  unsigned long canId = CAN.getCanId();

  /// unpacks ints from CAN buffer ///
  unsigned int id = buf[0];
  unsigned int p_int = (buf[1]<<8)|buf[2];
  //Serial.println(p_int);
  //delay(10);
  unsigned int v_int = (buf[3]<<4)|(buf[4]>>4);
  //Serial.println(v_int);
  //delay(10);
  unsigned int i_int = ((buf[4]&0xF)<<8)|buf[5];
  //Serial.println(i_int);
  //delay(10);
  /// converts uints to floats ///
  p_out = uint_to_float(p_int, P_MIN, P_MAX, 16);
  v_out = uint_to_float(v_int, V_MIN, V_MAX, 12);
  t_out = uint_to_float(i_int, T_MIN, T_MAX, 12);
  }


  unsigned int uint_to_float(unsigned int x_int, float x_min, float x_max, int bit)
  {/// converts an unsigned int to a float, given range and number of bits ///
  //Serial.println("conv1");
  //Serial.println(x_int);
  //Serial.print(x_min);
  //delay(20);
  float span = x_max - x_min;
  float offset = x_min;
  //Serial.println(offset);
  unsigned int pgg = 0;
  if(bit == 12)
  {
    float pgg = span*(x_int/4095);
    //Serial.println(pgg);
    pgg = pgg + offset;
    //delay(50);
  }
  if(bit == 16)
  {
    float pgg = span*(x_int/65535);
    //Serial.println(pgg);
    pgg = pgg + offset;
    //delay(50);
  }
  return pgg;
  }
*/