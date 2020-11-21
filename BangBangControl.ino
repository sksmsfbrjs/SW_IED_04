#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_SERVO 10
#define INTERVAL 20  // servo update interval

// configurable parameters
#define _DUTY_MIN 1210 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1430 // servo neutral position (90 degree)
#define _DUTY_MAX 1680 // servo full counterclockwise position (180 degree)

#define _SERVO_SPEED 90 // servo speed limit (unit: degree/second)
#define INTERVAL 20  // servo update interval

// global variables
unsigned long last_sampling_time; // unit: ms
int duty_chg_per_interval; // maximum duty difference per interval
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
Servo myservo;
int duty_target, duty_curr;
int a,b;
float ema;

void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO);
  duty_curr = _DUTY_MIN;
  myservo.writeMicroseconds(1430);
  
// initialize serial port
  Serial.begin(57600);

// convert angle speed into duty change per interval.

  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / 180.0) * (INTERVAL / 1000.0);
//SERVO_SPEED = 30 -> duty_chg_per_interval = 6

// initialize last sampling time
  last_sampling_time = 0;

  a = 69;
  b = 280;
}

float ir_distance(void){ // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0/(volt-9.0))-4.0) * 10.0;
  return val;
}

void loop() {
  float raw_dist = ir_distance();
  float dist_cali = 100 + 300.0 / (b - a) * (raw_dist - a);
  ema = 0.05 * dist_cali + 0.95 * ema;
// wait until next sampling time. 
// millis() returns the number of milliseconds since the program started. Will overflow after 50 days.
  if(millis() < last_sampling_time + INTERVAL) return;

// adjust duty_curr toward duty_target by duty_chg_per_interval
  if(255 > ema) {
    duty_curr += duty_chg_per_interval;
    if(duty_curr > 1680) duty_curr = 1680;
  }
  else {
    duty_curr -= duty_chg_per_interval;
    if(duty_curr < 1210) duty_curr = 1210;
  }

// update servo position
  myservo.writeMicroseconds(duty_curr);

// output the read value to the serial port
  Serial.print("min:0,max:500,dist:");
  Serial.print(raw_dist);
  Serial.print(",dist_cali:");
  Serial.print(dist_cali);
  Serial.print(",ema:");
  Serial.println(ema);

// update last sampling time
  last_sampling_time += INTERVAL;
}
