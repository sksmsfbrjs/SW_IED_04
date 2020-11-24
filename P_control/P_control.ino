#include <Servo.h>

// Arduino pin assignment
#define PIN_LED 9 
#define PIN_SERVO 10
#define PIN_IR A0

#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)

// sequence calibration
#define SEQ_SIZE 7

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
#define _DIST_ALPHA 0.35  //[3099] EMA 필터링을 위한 alpha 값

// Servo range
#define _DUTY_MIN 1640     //최저 서보 위치
#define _DUTY_NEU 1500     //중립 서보 위치
#define _DUTY_MAX 1380     //최대 서보 위치

// Servo speed control
#define _SERVO_ANGLE 30.0
#define _SERVO_SPEED 600.0

// Event periods
#define _INTERVAL_DIST 30
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100 

// PID parameters
#define _KP 0.06

//dist 100, 400mm 일때 값, 각자 a,b로 수정
#define a 69
#define b 260

// sensor values
float x[SEQ_SIZE] = {100.0, 150.0, 227.0, 280.0, 303.0, 323.0, 340.0};

// real values
float y[SEQ_SIZE] = {100.0, 150.0, 200.0, 250.0, 300.0, 350.0, 400.0};

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

void setup() {
  // initialize GPIO pins for LED and attach servo 
  myservo.attach(PIN_SERVO); // attach servo
  pinMode(PIN_LED,OUTPUT); // initialize GPIO pins
  
  // initialize global variables
  
  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);
  duty_curr = _DUTY_NEU;
  
  // initialize serial port
  Serial.begin(57600);
  
  // convert angle speed into duty change per interval.
    duty_chg_per_interval = (_DUTY_MIN - _DUTY_MAX) * (_SERVO_SPEED / 180.0 ) * (_INTERVAL_SERVO / 1000.0); 
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
    pterm = error_curr;

    iterm = 0; 
    dterm = 0;
    control = - _KP * pterm + iterm + dterm;

    // duty_target = f(duty_neutral, control)

    duty_target = _DUTY_NEU + control * ((control>0)?(_DUTY_MIN - _DUTY_NEU):(_DUTY_NEU - _DUTY_MAX))  * _SERVO_ANGLE / 180;

    // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target > _DUTY_MIN){
      duty_target = _DUTY_MIN;
    }
    else if(duty_target < _DUTY_MAX){
      duty_target = _DUTY_MAX;
    }
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
    Serial.print("dist_ir:");
    Serial.print(dist_raw);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

  }
}
//dist_cali
float ir_distance(void){ // return value unit: mm
  float value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return 300.0 / (b - a) * (value - a) + 100;
}

float under_noise_filter(void){ // 아래로 떨어지는 형태의 스파이크를 제거해주는 필터
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

//구간별 센서값 보정 코드 구현
float ir_distance_sequence(void){
  float value, real_value;
  float volt = float(analogRead(PIN_IR));
  value = ((6762.0/(volt-9.0))-4.0) * 10.0;
 
  int s = 0, e = SEQ_SIZE - 1, m;
  // binary search
  while(s <= e){
    m = (s + e) / 2;
    if(value < x[m]){
      e = m - 1;
    }
    else if(value > x[m+1]){
      s = m + 1;
    }
    else{
      break;
    }
  }
  // out of sequence range: set value to neutral position
  // 센서 전원 Off 시, 오차값 발생으로 구간에서 벗어났을 시 등 
  if(s > e){
    if(value > 2000.0 || value < -2000.0) real_value = _DIST_TARGET;
    else if(s == 0) real_value = _DIST_MIN; 
    else real_value = _DIST_MAX;
  }

  // calculate real values
  real_value = (y[m+1] - y[m]) / (x[m+1] - x[m] ) * (value - x[m]) + y[m];
  return real_value;
}
