/*unoでロボマス動かす　3A脇*/
/*角度制御、Motor_count等 追加 2B巻渕*/
/*ライブラリ*/
#include <SPI.h>//SPI通信に必要
#include <mcp2515.h>//速度制御器MCP2515を制御するのに必要
#include <MsTimer2.h>//プログラムを周期的に実行するのに必要
/*定義*/
#define Motor_count 4 //モータ数
/*データの送受信用構造体*/
struct can_frame canMsgReceive;
struct can_frame canMsgSend;
/*MCP2515を使うのに必要なオブジェクト*/
MCP2515 mcp2515(10);
/*各変数*/
short power[Motor_count]; // -16384 to 16384//速度制御器に送るトルクの強さ
float rotation[Motor_count]; //モータが起動してからどのくらい回ったか (度数法)0 - 360
short Speed_now[Motor_count];//現在のRPM
double Rotation_now[Motor_count];//現在の角度
short torque[Motor_count];//現在のトルク
int8_t temperature[Motor_count];//現在の温度
//--------------------------------------------------
void setup() {
  /*通信の初期設定*/
  Serial.begin(115200);//Serial通信開始
  SPI.begin();//SPI通信開始
  canMsgSend.can_id  = 0x200; // 通信のための接頭辞 これ+モーターID(1~4)
  canMsgSend.can_dlc = 8;//不明
  canMsgSend.data[0 - 7] = {0};//データ用配列
  mcp2515.reset();//MCP2515を初期化
  mcp2515.setBitrate(CAN_1000KBPS, MCP_8MHZ);//CAN通信のビットレートとMCP2515の動作周波数
  mcp2515.setNormalMode();//MCP2515をノーマルモードに
  /*関数の周期化*/
  MsTimer2::set(10, timer);//timer関数を10ms毎に実行
  MsTimer2::start();//同上
  /*シリアルプロッタ・モニタの凡例*/
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
  // CAN通信でデータを読み取る
  readValues();
  // CAN通信でデータを送る
  sendValues();
  //順にモータ比例ゲイン,モーター積分ゲイン,モーター微分ゲイン,角度制御比例ゲイン,角度制御微分ゲイン
  double kp_m = 2.5, ki = 0.5, kd = 0, kp_r = 2.0, kd_r = 0.0;
  double derivative_m[Motor_count],  derivative_r[Motor_count];//計算結果
  static double  integral[Motor_count];//同上
  //順に目標RPM,現在のRPM偏差,現在の角度偏差,過去のRPM偏差,過去のの角度偏差
  short Speed_target[Motor_count + 1] = {100, 100, 100, 100, 90}, error_now_M[Motor_count], error_now_R[Motor_count], error_old_M[Motor_count], error_old_R[Motor_count];
  double Rotation_Target[Motor_count] = {90};
  //角度制御(走行モータに実装する際は速さが少なくなるため角度制御プログラムをif文やユーザ関数で隔離すること)
  for (int i = 0; i < 4; i++) {
    Rotation_now[i] = rotation[i];
    error_now_R[i] = Rotation_Target[Motor_count] - Rotation_now[i];//P
    derivative_r[i] = (error_now_R[i] + error_old_R[i]);//D
    Speed_target[Motor_count] = kp_r * error_now_R[i] + derivative_r[i] * kd_r;//PD
    if (false) {//ここに角度制御に切り替える条件を書く
      Speed_target[i] = Speed_target[Motor_count];
    }
    //---ここまで---
    error_now_M[i] = Speed_target[i] - Speed_now[i];//P
    integral[i] += error_now_M[i];//I
    derivative_m[i] = (error_now_M[i] + error_old_M[i]);//D

    power[i] = kp_m * error_now_M[i] + ki * integral[i] + kd * derivative_m[i];//PID

    error_old_M[i] = error_now_M[i];
    /*シリアルモニタ・プロッタ*/
    Serial.print(Speed_target[i]);
    Serial.print(",");
    //Serial.print(Speed_now);
    //Serial.print(",");
    Serial.print(rotation[i]);
  }
  Serial.println();
  /*----------*/
  delay(1);

}

//--------------------------------------------------
void loop() {//使用しない
}
//--------------------------------------------------
void readValues() {
  if (mcp2515.readMessage(&canMsgReceive) == MCP2515::ERROR_OK) {//ちゃんと受信できたかを確認
    for (int i = 0; i < Motor_count; i++) {//CAN通信で送られてきたデータを正規化?しそれぞれの変数に格納
      if (canMsgReceive.can_id == canMsgSend.can_id + i + 1) { // Motor ID 1~Motor_count
        // rotation
        unsigned short r = (canMsgReceive.data[0] << 8) | (canMsgReceive.data[1] & 0xff);
        rotation[i] = (float)r / 8192 * 360;

        // Speed
        Speed_now[i] = (canMsgReceive.data[2] << 8) | (canMsgReceive.data[3] & 0xff);

        // torque
        torque[i] = (canMsgReceive.data[4] << 8) | (canMsgReceive.data[5] & 0xff);

        // temperature
        temperature[i] = canMsgReceive.data[6];
      }
    }
    //    if (canMsgReceive.can_id == canMsgSend.can_id + i + 1) { // Motor ID 1
    //      // rotation
    //      unsigned short r = (canMsgReceive.data[0] << 8) | (canMsgReceive.data[1] & 0xff);
    //      rotation[0] = (float)r / 8192 * 360;
    //
    //      // Speed
    //      Speed_now[0] = (canMsgReceive.data[2] << 8) | (canMsgReceive.data[3] & 0xff);
    //
    //      // torque
    //      torque[0] = (canMsgReceive.data[4] << 8) | (canMsgReceive.data[5] & 0xff);
    //
    //      // temperature
    //      temperature[0] = canMsgReceive.data[6];
    //    } else if (canMsgReceive.can_id == 0x202) { // Motor ID 2
    //      // rotation
    //      unsigned short r = (canMsgReceive.data[0] << 8) | (canMsgReceive.data[1] & 0xff);
    //      rotation[1] = (float)r / 8192 * 360;
    //
    //      // Speed
    //      Speed_now[1] = (canMsgReceive.data[2] << 8) | (canMsgReceive.data[3] & 0xff);
    //
    //      // torque
    //      torque[1] = (canMsgReceive.data[4] << 8) | (canMsgReceive.data[5] & 0xff);
    //
    //      // temperature
    //      temperature[1] = canMsgReceive.data[6];
    //    } else if (canMsgReceive.can_id == 0x203) { // Motor ID 3
    //      // rotation
    //      unsigned short r = (canMsgReceive.data[0] << 8) | (canMsgReceive.data[1] & 0xff);
    //      rotation[2] = (float)r / 8192 * 360;
    //
    //      // Speed
    //      Speed_now[2] = (canMsgReceive.data[2] << 8) | (canMsgReceive.data[3] & 0xff);
    //
    //      // torque
    //      torque[2] = (canMsgReceive.data[4] << 8) | (canMsgReceive.data[5] & 0xff);
    //
    //      // temperature
    //      temperature[2] = canMsgReceive.data[6];
    //    } else if (canMsgReceive.can_id == 0x204) { // Motor ID 4
    //      // rotation
    //      unsigned short r = (canMsgReceive.data[0] << 8) | (canMsgReceive.data[1] & 0xff);
    //      rotation[3] = (float)r / 8192 * 360;
    //
    //      // Speed
    //      Speed_now[3] = (canMsgReceive.data[2] << 8) | (canMsgReceive.data[3] & 0xff);
    //
    //      // torque
    //      torque[3] = (canMsgReceive.data[4] << 8) | (canMsgReceive.data[5] & 0xff);
    //
    //      // temperature
    //      temperature[3] = canMsgReceive.data[6];
    //    }
  }
}
//--------------------------------------------------
void sendValues() {
  int j = 0;
  for (int i = 0; i < Motor_count; i++) {//CAN通信で送るトルク量を送信用配列に格納
    canMsgSend.data[j] = power[i] >> 8 & 0xff;
    j++;
    canMsgSend.data[j] = power[i] & 0xff;
    j++;
  }
  //  canMsgSend.data[0] = power[0] >> 8 & 0xff;
  //  canMsgSend.data[1] = power[0] & 0xff;
  //  canMsgSend.data[2] = power[1] >> 8 & 0xff;
  //  canMsgSend.data[3] = power[1] & 0xff;
  //  canMsgSend.data[4] = power[2] >> 8 & 0xff;
  //  canMsgSend.data[5] = power[2] & 0xff;
  //  canMsgSend.data[6] = power[3] >> 8 & 0xff;
  //  canMsgSend.data[7] = power[3] & 0xff;
  mcp2515.sendMessage(&canMsgSend);//送信
}
