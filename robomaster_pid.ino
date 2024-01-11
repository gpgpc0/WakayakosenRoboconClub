/*unoでロボマス動かす　3A脇*/
#include <SPI.h>
#include <mcp2515.h>
#include <MsTimer2.h>

struct can_frame canMsgReceive;
struct can_frame canMsgSend;

MCP2515 mcp2515(10);

short power[3] = {0, 0, 0}; // -16384 to 16384
float rotation = 0; // 0 - 360
short Speed_now = 0;
double Rotation_now = 0;
short torque = 0;
int8_t temperature = 0;


//--------------------------------------------------
void setup() {
  Serial.begin(115200);
  SPI.begin();
  canMsgSend.can_id  = 0x200; // for Motor ID 1 to 4
  canMsgSend.can_dlc = 8;
  canMsgSend.data[0] = 0;
  canMsgSend.data[1] = 0;
  canMsgSend.data[2] = 0;
  canMsgSend.data[3] = 0;
  canMsgSend.data[4] = 0;
  canMsgSend.data[5] = 0;
  canMsgSend.data[6] = 0;
  canMsgSend.data[7] = 0;

  mcp2515.reset();
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  MsTimer2::set(10, timer);
  MsTimer2::start();
  Serial.print("目標値");
  Serial.print(",");
  /*
    Serial.print("Speed");
    Serial.print(",");
  */
  Serial.println("rotation:");
}

//--------------------------------------------------
void timer() {
  // read values via can
  readValues();
  // send values via can
  sendValues();

  //Serial.println(power[1]);

  double kp_m = 2.5, ki = 0.5, kd = 0, derivative_m, kp_r = 2.0, kd_r = 0.0, derivative_r;
  static double  integral;
  //角度制御(走行モータに実装する際は速さが少なくなるため角度制御プログラムをif文やユーザ関数で隔離すること)
  Rotation_now = rotation;
  short Speed_target = 90, error_now_M, error_now_R, error_old_M, error_old_R;//交流会5000yen
  double Rotation_Target = 90;
  error_now_R = Rotation_Target - Rotation_now;
  derivative_r = (error_now_R + error_old_R);
  Speed_target = kp_r * error_now_R + derivative_r * kd_r;
  //---ここまで---
  error_now_M = Speed_target - Speed_now;
  integral += error_now_M;//(error_now + error_old);
  derivative_m = (error_now_M + error_old_M);

  power[1] = kp_m * error_now_M + ki * integral + kd * derivative_m;

  error_old_M = error_now_M;
  Serial.print(Speed_target);
  Serial.print(",");
  //Serial.print(Speed_now);
  //Serial.print(",");
  Serial.println(rotation);//度数法
  delay(1);

}

//--------------------------------------------------
void loop() {
}

//--------------------------------------------------
void readValues() {
  if (mcp2515.readMessage(&canMsgReceive) == MCP2515::ERROR_OK) {

    if (canMsgReceive.can_id == 0x201) { // Motor ID 1
      // rotation
      unsigned short r = (canMsgReceive.data[0] << 8) | (canMsgReceive.data[1] & 0xff);
      rotation = (float)r / 8192 * 360;

      // Speed
      Speed_now = (canMsgReceive.data[2] << 8) | (canMsgReceive.data[3] & 0xff);

      // torque
      torque = (canMsgReceive.data[4] << 8) | (canMsgReceive.data[5] & 0xff);

      // temperature
      temperature = canMsgReceive.data[6];


      //Serial.print("rotation:");
      //Serial.print(rotation);
      //Serial.print(" ");

      //      Serial.print("Speed:");
      //      Serial.print(Speed_now);
      //      Serial.println(",");

      //      Serial.print("torque:");
      //      Serial.print(torque);
      //      Serial.print(" ");
      //
      //      Serial.print("temperature:");
      //      Serial.print(temperature);
      //      Serial.print(" ");
      //
      //      Serial.println();
    }
  }
}

//--------------------------------------------------
void sendValues() {
  canMsgSend.data[0] = power[1] >> 8 & 0xff;
  canMsgSend.data[1] = power[1] & 0xff;
  mcp2515.sendMessage(&canMsgSend);
}
