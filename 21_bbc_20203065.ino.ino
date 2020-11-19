#include <Servo.h>

// Arduino pin assignment
#define PIN_IR A0
#define PIN_LED 9
#define PIN_SERVO 10

#define _DUTY_MIN 1200
#define _DUTY_NUE 1350
#define _DUTY_MAX 1500

int a, b;
int duty_curr;
Servo myservo;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_NUE);
  
// initialize serial port
  Serial.begin(57600);

  a = 83;
  b = 301;

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
  Serial.print("min:0,max:500,dist:");
  Serial.print(dist_cali);
  if(250 >= dist_cali){
    digitalWrite(PIN_LED, 1);
    duty_curr = _DUTY_MAX;
  }
  else{
    digitalWrite(PIN_LED, 1);
    duty_curr = _DUTY_MIN;
  }
  myservo.writeMicroseconds(duty_curr);
  Serial.print(",duty_curr:");
  Serial.println(duty_curr);
}
