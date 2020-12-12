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

#define _DIST_ALPHA 0.1 // EMA 필터 가중치

#define _DUTY_MIN 1000 // 서보 각도 최소값
#define _DUTY_NEU 1420 // 서보 수평
#define _DUTY_MAX 2000 // 서보 각도 최대값

#define _SERVO_ANGLE 30 //[3030] servo angle limit 실제 서보의 동작크기 
                        //[3023] 서보모터의 작동 범위(단위 : degree)

#define _SERVO_SPEED 1000 //[3040] 서보의 각속도(초당 각도 변화량)

#define _INTERVAL_SERVO 10 //서보 갱신 간격
#define _INTERVAL_SERIAL 100 // 시리얼 플로터 갱신 간격

#define _KP 0.7 //[3039] 비례 제어의 상수 값
#define _KD 36
#define _KI 0.0041

#define _a 70
#define _b 450

#define _INTERVAL_DIST 30  // DELAY_MICROS * samples_num^2 의 값이 최종 거리측정 인터벌임. 넉넉하게 30ms 잡음.
#define DELAY_MICROS  1500 // 필터에 넣을 샘플값을 측정하는 딜레이(고정값!)
#define EMA_ALPHA 0.08    // EMA 필터 값을 결정하는 ALPHA 값. 작성자가 생각하는 최적값임.
float ema_dist=0;            // EMA 필터에 사용할 변수
float filtered_dist;       // 최종 측정된 거리값을 넣을 변수. loop()안에 filtered_dist = filtered_ir_distance(); 형태로 사용하면 됨.
float samples_num = 3;     // 스파이크 제거를 위한 부분필터에 샘플을 몇개 측정할 것인지. 3개로 충분함! 가능하면 수정하지 말 것.

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
const float coE[] = {0.0000026, -0.0023484, 1.5215775, 4.8144088};





void setup() {
  iterm = -55;
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

  error_prev = 3;
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
  // get a distance reading from the distance sensor  // [3037]
     float x = ir_distance_filtered();
     dist_ema = coE[0] * pow(x, 3) + coE[1] * pow(x, 2) + coE[2] * x + coE[3];

  // PID control logic
    error_curr = dist_target - dist_ema; //[3034] 목표위치와 실제위치의 오차값 
    pterm = _KP * error_curr; //[3034]
    dterm = _KD * (error_curr - error_prev);
    iterm += _KI * error_curr;
    control = pterm + dterm + iterm; //[3034]

  // duty_target = f(duty_neutral, control)
    duty_target = duty_neutral + control; //[3034]

  // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target>_DUTY_MAX) duty_target = _DUTY_MAX;
    if(duty_target<_DUTY_MIN) duty_target = _DUTY_MIN;

    error_prev = error_curr;
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
}

float ir_distance(void){ // return value unit: mm
  float val; //[3031] 
  float volt = float(analogRead(PIN_IR)); //[3031]
  val = ((6762.0/(volt-9.0))-4.0) * 10.0; //[3031]
  return val; //[3031]
}

float ir_distance_filtered(void){ // return value unit: mm
 // under_noise_filter를 통과한 값을 upper_nosie_filter에 넣어 최종 값이 나옴.
  int currReading;
  int lowestReading = 1024;
  for (int i = 0; i < samples_num; i++) {
    currReading = under_noise_filter();
    if (currReading < lowestReading) { lowestReading = currReading; }
  }
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
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
