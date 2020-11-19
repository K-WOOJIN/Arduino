#include <Servo.h>
// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10
int a, b; // unit: mm
float alpha;
Servo myservo;
void setup() {
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(1460);
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
// initialize serial port
  Serial.begin(57600);
  a = 70;
  b = 280;
  alpha = 0.05;
}
float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}
void loop() {
  float raw_dist = ir_distance();
  // ema 필터를 적용한 값을 사용. 실제 거리 25.5cm일 때 필터값 10
  float dist_ema = alpha * raw_dist + (1 - alpha) * dist_ema;
  float dist_cali = 100 + 275.0 / (b - a) * (dist_ema - a);
  Serial.print("min:0,max:500,dist:");
  Serial.print(dist_ema);
  Serial.print(",dist_cali:");
  Serial.println(dist_cali);
  if (dist_ema > 10) myservo.writeMicroseconds(1380);
  else myservo.writeMicroseconds(1540);
  delay(20);
}
