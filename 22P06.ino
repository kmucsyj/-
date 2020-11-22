#include <Servo.h>

////////////
// 상수 값 //
////////////

#define PIN_LED 9 // LED

#define PIN_SERVO 10 // 서보
#define PIN_IR A0 // 적외선 센서

#define _DIST_TARGET 255 // 레일플레이트 위 목표 지점 25.5cm
#define _DIST_MIN 100
#define _DIST_MAX 410

#define _DIST_ALPHA 0.3 // EMA 필터 가중치

#define _DUTY_MIN 1000 // 서보 각도 최소값
#define _DUTY_NEU 1500 // 서보 수평
#define _DUTY_MAX 2000 // 서보 각도 최대값

#define _SERVO_ANGLE 30 //[3030] servo angle limit 실제 서보의 동작크기 
                        //[3023] 서보모터의 작동 범위(단위 : degree)

#define _SERVO_SPEED 100 //[3040] 서보의 각속도(초당 각도 변화량)

#define _INTERVAL_DIST 20 //적외선 센서 측정 간격
#define _INTERVAL_SERVO 10 //서보 갱신 간격
#define _INTERVAL_SERIAL 50 // 시리얼 플로터 갱신 간격

#define _KP 0.6//[3039] 비례 제어의 상수 값

#define _a 70
#define _b 400

//////////////
// 전역 변수 //
/////////////

Servo myservo;

float dist_target; // 목표 거리
float dist_raw, dist_ema; // 적외선센서 값, ema필터 값
float dist_min, dist_max;

unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;   //[3039] interval간격으로 동작 시행을 위한 정수 값
bool event_dist, event_servo, event_serial; //[3023] 적외선센서의 거리측정, 서보모터, 시리얼의 이벤트 발생 여부

int duty_chg_per_interval;  //[3039] interval 당 servo의 돌아가는 최대 정도
int duty_target, duty_curr;      //[3030]servo 목표 위치, servo 현재 위치
int duty_neutral; // 서보 수평

float error_curr, error_prev, control, pterm, dterm, iterm; 
// [3042] 현재 오차값, 이전 오차값, ? , p,i,d 값

float ema_b, ema_k;





void setup() {
  pinMode(PIN_LED,OUTPUT);
  myservo.attach(PIN_SERVO);

  dist_min = _DIST_MIN; 
  dist_max = _DIST_MAX;
  dist_target = _DIST_TARGET; 
  duty_neutral = _DUTY_NEU;

  myservo.writeMicroseconds(_DUTY_NEU);

  Serial.begin(57600);  //[3039] 시리얼 모니터 속도 지정

  duty_chg_per_interval =(float)(_DUTY_MAX - _DUTY_MIN) * (_SERVO_SPEED / _SERVO_ANGLE) * (_INTERVAL_SERVO / 1000.0);
  duty_curr = 1550;
}



void loop() {
/////////////////////
// Event generator //
/////////////////////

  unsigned long time_curr = millis();
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
        last_sampling_time_dist += _INTERVAL_DIST;
        event_dist = true;
  }
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
        last_sampling_time_servo += _INTERVAL_SERVO;
        event_servo = true;
  }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
        last_sampling_time_serial += _INTERVAL_SERIAL;
        event_serial = true;
  }
  
////////////////////
// Event handlers //
////////////////////

 if(event_dist) {
     event_dist = false;    // [3037]
  // get a distance reading from the distance sensor
     dist_raw = ir_distance();
     dist_ema = ir_distance_filtered();   // [3037]

  // PID control logic
    error_curr = dist_target - dist_ema; //[3034] 목표위치와 실제위치의 오차값 
    pterm =  error_curr; //[3034]
    control = _KP * pterm; //[3034]

  // duty_target = f(duty_neutral, control)
    duty_target = duty_neutral + control; //[3034]

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target>_DUTY_MAX) duty_target = _DUTY_MAX;
    if(duty_target<_DUTY_MIN) duty_target = _DUTY_MIN;
  }


  if(event_servo) {
      event_servo = false; //[3034]
  
      // adjust duty_curr toward duty_target by duty_chg_per_interval 
       if(duty_target > duty_curr) {
          duty_curr += duty_chg_per_interval;
          if(duty_curr > duty_target) duty_curr = duty_target;
        }
        else {
          duty_curr -= duty_chg_per_interval;
          if(duty_curr < duty_target) duty_curr = duty_target;
    }//[3034] 서보의 현재 위치가 서보 목표 위치에 도달하기전이면 도달하기전까지 duty_chg_per_interval를 증감시킴. 만약 서보 현재 위치가 서보 목표 위치를 벗어난다면 서보 현재 위치를 서보 목표 위치로 고정
  
      // update servo position
  myservo.writeMicroseconds(duty_curr);//[3034] 

  }



  if(event_serial) {
    event_serial = false;               // [3030]
    Serial.print("dist_raw:");
    Serial.print(dist_raw);
    Serial.print(",dist_raw:");
    Serial.print(dist_ema);
    Serial.print(",pterm:");
    Serial.print(map(pterm,-1000,1000,510,610));
    Serial.print(",duty_target:");
    Serial.print(map(duty_target,1000,2000,410,510));
    Serial.print(",duty_curr:");
    Serial.print(map(duty_curr,1000,2000,410,510));
    Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");
  }
}

float ir_distance(void){ // return value unit: mm
  float val; //[3031] 
  float volt = float(analogRead(PIN_IR)); //[3031]
  val = ((6762.0/(volt-9.0))-4.0) * 10.0; //[3031]
  return val; //[3031]
}

float ir_distance_filtered(void){ // return value unit: mm
  float raw_dist = ir_distance(); // for now, just use ir_distance() without noise filter.
  //float dist_cali = 100 + 300.0 / (_b - _a) * (raw_dist - _a);
  float dist_cali = map(raw_dist,70,450,100,420);
  ema_k = (_DIST_ALPHA * dist_cali) + ((1 - _DIST_ALPHA)* ema_b);
  ema_b = ema_k;
  
  return ema_k;
}
