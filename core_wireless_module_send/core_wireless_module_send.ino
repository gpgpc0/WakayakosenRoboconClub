//#include <MsTimer2.h>
//#include <SoftwareSerial.h>

//SoftwareSerial mySerial(2, 3);

void setup() {
  Serial.begin(115200);
  //mySerial.begin(115200);
  //MsTimer2::set(200, module);
  //MsTimer2::start();
}

void loop() {
  //Serial.println("05,06,04,03,0f,0e,0d");
  
  Serial.write(06);
  Serial.print(',');
  Serial.write(05);
  Serial.print(',');
  Serial.write(04);
  Serial.print(',');
  Serial.write(03);
  Serial.print(',');
  Serial.write(15);   //0f
  Serial.print(',');
  Serial.write(14);   //0e
  Serial.print(',');
  Serial.write(13);   // 0d
  Serial.println();

  //Serial.write(06);
  delay(200);
}

void module() {
  //mySerial.println("05,06,04,03,0f,0e,0d");
}
