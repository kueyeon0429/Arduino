#include <Servo.h>

/////////////////////////////
// Configurable parameters //
/////////////////////////////

// Arduino pin assignment
#define PIN_LED 9              // LED를 아두이노 GPIO 9번 핀에 연결  
#define PIN_SERVO 10           // 서보모터를 아두이노 GPIO 10번 핀에 연결
#define PIN_IR A0              // 적외선 센서를 아두이노 아날로그 A0핀에 연결

// Framework setting
#define _DIST_TARGET 255       // 목표하는 위치
#define _DIST_MIN 100          // 거리의 최솟값이 100mm
#define _DIST_MAX 410          // 거리의 최대값 410mm

// Distance sensor
#define _DIST_ALPHA 0.0        // 센서 보정정도

#define DELAY_MICROS 1500
#define EMA_ALPHA 0.35        

// Servo range
#define _DUTY_MIN 1000         // 서보의 최소각도를 microseconds로 표현
#define _DUTY_NEU 1400         // 레일플레이트 중립위치를 만드는 서보 duty값
#define _DUTY_MAX 1800         // 서보의 최대각도의 microseconds의 값

// Servo speed control
#define _SERVO_ANGLE 30        // 최대 가동범위에 따른 목표 서보 회전각
#define _SERVO_SPEED 30        // 서보 속도를 30으로

// Event periods
#define _INTERVAL_DIST 5      // 적외선센서 업데이트 주기
#define _INTERVAL_SERVO 5     // 서보 업데이트 주기
#define _INTERVAL_SERIAL 50   // 시리얼 플로터 갱신 속도

// PID parameters
#define _KP 1.1   // P 이득 비율
#define _KI 0.0   // I 이득 비율
#define _KD 0.0 // D 이득 비율


//////////////////////
// global variables //
//////////////////////

// Servo instance
Servo myservo;   // 서보 객체 생성

float ema_dist = 0;
float filtered_dist;
float samples_num = 3;

// Distance sensor
float dist_target;
float dist_raw, dist_ema, dist_cali;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial; // 이벤트별 이번 루프에서 업데이트 여부

// Servo speed control
int duty_chg_per_interval; // 서보 속도 제어를 위한 변수 선언
int duty_target, duty_curr; // 목표 duty, 현재 duty
int duty_neutral = _DUTY_NEU;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;
   // PID 제어를 위한 현재 오차, 이전오차, 컨트롤(?), p 값, d 값, i 값 변수 선언

int a = 83;
int b = 301;


void setup() {
  // initialize GPIO pins for LED and attach servo
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);

  // initialize global variables
  duty_target, duty_curr = _DIST_MIN;
  last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial = 0;
  dist_raw, dist_ema = _DIST_MIN;
  pterm = iterm = dterm = 0;

  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize serial port
  Serial.begin(57600);

  event_dist = event_serial = false;

  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (_DUTY_MAX - _DUTY_MIN)/(float)(_SERVO_ANGLE) * (_SERVO_SPEED /1000.0)*_INTERVAL_SERVO;
     // 설정한 서보 스피드에 따른 duty change per interval 값을 변환

}


void loop() {
  /////////////////////
  // Event generator //
  /////////////////////
  unsigned long time_curr = millis();   // 이벤트 업데이트 주기 계산을 위한 현재 시간
  if(time_curr >= last_sampling_time_dist + _INTERVAL_DIST){
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
    }
    
  if(time_curr >= last_sampling_time_servo + _INTERVAL_SERVO){
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
    }
  if(time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL){
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
    }


  ////////////////////
  // Event handlers //
  ////////////////////
  
  if(event_dist){
    event_dist = false;  // 업데이트 대기
    // get a distance reading from the distance sensor
    dist_raw = filtered_ir_distance();
    dist_cali = 100 + 300.0 / (b - a) * (dist_raw - a);
    dist_ema = _DIST_ALPHA * dist_cali + (1-_DIST_ALPHA)*dist_ema;

    // PID control logic
    error_curr = _DIST_TARGET - dist_cali; // 오차 계산
    pterm = error_curr; // p 값은 오차
    control = _KP*pterm + _KI*iterm + _KD*dterm; // control값, i와 d는 현재 0

    // duty_target = f(duty_neutral, control)
    duty_target = duty_neutral+control;
    //duty_target = duty_neutral*(1+control);
       // control 값이 다 합해서 1이 되도록 되어있다면, 중립 위치에 컨트롤 값 만큼의 비율을 더해 목표위치를 정한다.

    //duty_target = _DUTY_NEU + (control - 0) * (control > 0 ? _DUTY_MAX - _DUTY_NEU : _DUTY_NEU - _DUTY_MIN);
       // NEU 에다가 위아래로 control 에 맞게 조절한 값
      
    // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if(duty_target < _DUTY_MIN)   //  양극값 넘어가는 경우 극값으로 제한
    {
      duty_target = _DUTY_MIN;
      }
    if(duty_target > _DUTY_MAX)
    {
      duty_target = _DUTY_MAX;
      }
    }
    
    if(event_servo){
      event_servo = false;
      // adjust duty_curr toward duty_target by duty_chg_per_interval
      if(duty_target > duty_curr){
        duty_curr += duty_chg_per_interval;
        if(duty_curr > duty_target)duty_curr = duty_target;
      }
      else{
        duty_curr -= duty_chg_per_interval;
        if(duty_curr < duty_target) duty_curr = duty_target;
      }

    // update servo position
    myservo.writeMicroseconds((int)duty_curr);

    event_servo = false;
    }

    if(event_serial) {
      event_serial = false; // 이벤트 주기가 왔다면 다시 false로 만들고 이벤트를 수행
      Serial.print("dist_ir:");
      Serial.print(dist_cali);
      Serial.print(",pterm:");
      Serial.print(map(pterm,-1000,1000,510,610));
      Serial.print(",duty_target:");
      Serial.print(map(duty_target,1000,2000,410,510));
      Serial.print(",duty_curr:");
      Serial.print(map(duty_curr,1000,2000,410,510));
      Serial.println(",Min:100,Low:200,dist_target:255,High:310,Max:410");

      event_serial = false; // event_serial false로 변경
      }
}


float ir_distance(void){ // return value unit: mm
       float val; // 변수 val 선언
       float volt = float(analogRead(PIN_IR)); // volt변수에 적외선 센서 측정값 입력
       val = ((6762.0/(volt - 9.0)) - 4.0) * 10.0; // volt 값 보정
       return val; // val 리턴
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
  // eam 필터 추가
  ema_dist = EMA_ALPHA*lowestReading + (1-EMA_ALPHA)*ema_dist;
  return ema_dist;
}
