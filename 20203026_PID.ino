#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 400
// Distance sensor
#define _DIST_ALPHA 0.5

// Servo range 
#define _DUTY_MIN 1340
#define _DUTY_NEU 1460
#define _DUTY_MAX 1680

// Servo speed control
#define _SERVO_ANGLE 50
#define _SERVO_SPEED 250
// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100
#define DELAY_MICROS 1500
#define EMA_ALPHA 0.5
// PID parameters
#define _KP 1.26
#define _KD 62
#define _KI 0.17
#define _ITERM_MAX 4

//////////////////////
// global variables //
//////////////////////

// Servo instance 
Servo myservo;    

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema;
// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;   //[3039] interval간격으로 동작 시행을 위한 정수 값
bool event_dist, event_servo, event_serial; //[3023] 적외선센서의 거리측정, 서보모터, 시리얼의 이벤트 발생 여부
// Servo speed control
int duty_chg_per_interval;  //[3039] interval 당 servo의 돌아가는 최대 정도
int duty_target, duty_curr, duty_neutral;      //[3030]servo 목표 위치, servo 현재 위치
float ema_dist=0;            // EMA 필터에 사용할 변수
float filtered_dist, final_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3; 
float x;
// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm; 
// [3042] 현재 오차값, 이전 오차값, ? , p,i,d 값
const float coE[] = {-0.0000037, 0.0040163, -0.1624214, 126.4923558};
void setup() {
  pterm = dterm = 0;
  iterm = 1;
  duty_neutral = 1460;
  event_servo = false;
  event_dist = false;
  event_servo = false;
// initialize GPIO pins for LED and attach servo 
  pinMode(PIN_LED,OUTPUT);           //[3030]LED를 연결[3027]
  myservo.attach(PIN_SERVO);  //[3039]servo를 연결
  myservo.writeMicroseconds(1460);
// initialize global variables
  float dist_min = dist_raw = dist_ema = _DIST_MIN;                      //[3030] 측정값의 최소값
          //[3023] 측정가능 거리의 최소값
  float dist_max= _DIST_MAX;    //[3032] 측정값의 최대값
          //[3023] 측정가능 거리의 최대값
  dist_target = _DIST_TARGET;           //[3023] 목표 거리 위치
// move servo to neutral position
// initialize serial port
Serial.begin(57600);  //[3039] 시리얼 모니터 속도 지정 

// convert angle speed into duty change per interval.
  duty_chg_per_interval =(float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0); //[3039] 
/*
_SERVO_ANGLE/_SERVO_SPEED=
(_DUTY_MAX - _DUTY_MIN_)* INTERVAL_SERVO/duty_chg_per_interval = 
interval 반복횟수*INTERVAL_SERVO=
_SERVO_ANGLE만큼 돌아가는데 걸리는 시간 
*/
  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;
}
  

void loop() {
/////////////////////
// Event generator //
/////////////////////
  unsigned long current_time = millis();
  if (current_time >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (current_time >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (current_time >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }
////////////////////
// Event handlers //
////////////////////

  if(event_dist) {
    event_dist = false;
  // get a distance reading from the distance sensor
    x = ir_distance_filtered();
    dist_raw = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
    dist_ema = _DIST_ALPHA * dist_raw + (1 - _DIST_ALPHA) * dist_ema;
    error_curr = dist_target - dist_ema;
    pterm =  _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    if(abs(iterm) > _ITERM_MAX) iterm = 0;
    control = pterm + dterm + iterm;
  // duty_target = f(duty_neutral, control)
    duty_target = duty_neutral + control;
  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;
    else if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
    error_prev = error_curr;
  }
  
  if(event_servo) {
    event_servo = false;
    
    //if(millis() < last_sampling_time_servo + _INTERVAL_SERVO) return;
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
      }
    myservo.writeMicroseconds(duty_curr);//[3034] 
  }
  
  if(event_serial) {
    event_serial = false;

    Serial.print("IR:");
    Serial.print(dist_ema);
    Serial.print(",T:");
    Serial.print(dist_target);
    Serial.print(",P:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",D:");
    Serial.print(map(dterm,-1000,1000,510,610));
    Serial.print(",I:");
    Serial.print(map(iterm,-1000,1000,510,610));
    Serial.print(",DTT:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",DTC:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",-G:245,+G:265,m:0,M:800");
    }
    //last_sampling_time_servo += _INTERVAL_SERVO;
    //last_sampling_time_serial += _INTERVAL_SERIAL;
    //last_sampling_time_dist += _INTERVAL_DIST;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

float under_noise_filter(void) { // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_distance();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}

float ir_distance_filtered(void){ // return value unit: mm
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA * lowestReading + (1-EMA_ALPHA) * ema_dist;
  return ema_dist;
}
