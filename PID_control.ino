#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9 
#define PIN_SERVO 10
#define PIN_IR A0

#define DELAY_MICROS 1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.35  //[3099] EMA 필터링을 위한 alpha 값

// Servo range
#define _DUTY_MIN 1660     //최저 서보 위치
#define _DUTY_NEU 1500     //중립 서보 위치
#define _DUTY_MAX 1300     //최대 서보 위치

// Servo speed control
#define _SERVO_SPEED 650.0

// Event periods
#define _INTERVAL_DIST 30
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 2
#define _KD 59
#define _KI 0.005

//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_ema; //측정된 값과 ema 필터를 적용한 값

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;  //[3104] 각 event의 진행 시간 저장 변수 
bool event_dist, event_servo, event_serial;  // 각 event의 시간체크를 위한 변수 (ex_20초 주기 >> 0초(True,시작), 10초(False), 20초(True))

// Servo speed control
int duty_chg_per_interval; // 주기 당 서보 duty값 변화량
int duty_target, duty_curr; // 목표 위치와 현재 위치

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

//error_curr: 현재 측정값과 목표값의 차이
//error_prev: 직전에 구한 차이로, P제어에서는 사용하지 않을 것임
//control: PID제어의 결과로 얻은 제어값
//pterm: Proportional term, 현재 상태의 error값으로부터 얻은 Proportional gain을 저장하는 변수

float ema_dist=0;            // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3;
// global variables
const float coE[] = {0.0000006, 0.0020380, 0.0201167, 89.8742929};
int a, b; // unit: mm

void setup() {
  // initialize GPIO pins for LED and attach servo 
  myservo.attach(PIN_SERVO); // attach servo
  pinMode(PIN_LED,OUTPUT); // initialize GPIO pins
  
  // initialize global variables
  
  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);
  duty_curr = _DUTY_NEU;
  dist_target = _DIST_TARGET;
  // initialize serial port
  Serial.begin(57600);
  
  // convert angle speed into duty change per interval.
    duty_chg_per_interval = (_DUTY_MIN - _DUTY_MAX) * (_SERVO_SPEED / 180.0 ) * (_INTERVAL_SERVO / 1000.0); 

  a = 69;
  b = 290;

  pterm = 0;
  iterm = 0;
  dterm = 0;
}
  

void loop() {
  /////////////////////
  // Event generator //
  ///////////////////// 
  unsigned long time_curr = millis();
  
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
      last_sampling_time_dist += _INTERVAL_DIST;
      event_dist = true;
  }
  
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
      last_sampling_time_servo += _INTERVAL_SERVO;
      event_servo = true;
  }
  
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL ){
      last_sampling_time_serial += _INTERVAL_SERIAL;
      event_serial = true;
  }
  
  ////////////////////
  // Event handlers //
  ////////////////////

  if(event_dist) {
    event_dist = false;
    dist_ema = filtered_ir_distance(); // get a distance reading from the distance sensor

    // PID control logic
    error_curr = dist_ema - _DIST_TARGET;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    control = -(dterm + pterm + iterm);

    // duty_target = f(duty_neutral, control)

    duty_target = _DUTY_NEU + control;

    // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target > _DUTY_MIN) {duty_target = _DUTY_MIN;}
    if(duty_target < _DUTY_MAX){duty_target = _DUTY_MAX;}
    
    // update error_prev
    error_prev = error_curr;
  }
  
  if(event_servo) {
    event_servo = false;
    // adjust duty_curr toward duty_target by duty_chg_per_interval
    
    if(duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if(duty_curr > duty_target) duty_curr = duty_target;
    } else {
      duty_curr -= duty_chg_per_interval;
      if(duty_curr < duty_target) duty_curr = duty_target;
    }
    // update servo position
    myservo.writeMicroseconds(duty_curr);
  }   

  
  if(event_serial) {
   event_serial = false;
   Serial.print("IR:");
   Serial.print(dist_ema);
   Serial.print(",T:");
   Serial.print("255");
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
}

//dist_cali
float ir_distance(void){ // return value unit: mm
  float value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return 300.0 / (b - a) * (value - a) + 100;
}

float ir_dist_filter() {
  float x = ir_distance();
  dist_raw = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];
  return dist_raw;
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
  int currReading;
  int largestReading = 0;
  for (int i = 0; i < samples_num; i++) {
    currReading = ir_dist_filter();
    if (currReading > largestReading) { largestReading = currReading; }
    // Delay a short time before taking another reading
    delayMicroseconds(DELAY_MICROS);
  }
  return largestReading;
}
float filtered_ir_distance(void){ // 아래로 떨어지는 형태의 스파이크를 제거 후, 위로 치솟는 스파이크를 제거하고 EMA필터를 적용함.
  // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  ema_dist = _DIST_ALPHA * lowestReading + (1-_DIST_ALPHA)*ema_dist;
  return ema_dist;
}
