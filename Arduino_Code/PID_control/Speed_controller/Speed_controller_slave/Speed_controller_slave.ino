#include <ros.h>
#include <std_msgs/Float32.h>

double Kp = 5, Ki = 0.1, Kd = 0.15;
double error, error_sum = 0, d_error, prev_error;
double curr_v;
double target_v;
double motor_val;
boolean Direction;
const double PPR = 224.4;
const double WHEEL_DIA = 0.11; //outer diameter of the wheel is 110 mm
int pulse_count = 0, prev_pulse = 0;
const byte encoder0pinA = 2;//A pin -> the interrupt pin 0
const byte encoder0pinB = 3;//B pin -> the digital pin 3
byte encoder0PinALast;
int EN_B = 5;
int IN3 = 7;
int IN4 = 6;
unsigned long time1 = 0;
unsigned long time2 = 0;

ros::NodeHandle nh_slave;

std_msgs::Float32 curr_speed;

void chatterCallBack(const std_msgs::Float32& curr_speed)
{
  target_v = (double) curr_speed.data;
}

ros::Subscriber<std_msgs::Float32> sub("chatter", 1000, chatterCallBack);


void setup() {
  Serial.begin (9600); //Starting the serial communication at 9600 baud rate
  //Initializing the motor pins as output
  pinMode(EN_B, OUTPUT);
  pinMode(IN3, OUTPUT);  
  pinMode(IN4, OUTPUT);
  EncoderInit();//Initialize the module
//  nh_slave.initNode();
 while (!Serial) {
  ;
 }
  time1 = micros();
}

byte buffer[16];

void loop() {  
// delay(50);
// nh_slave.subscribe(sub);

 if (Serial.available()== 4) {
//  int size = Serial.readyBytesUntil('\n', buffer, 4);
  byte byte_v[4];
  byte_v[0] = Serial.read();
  byte_v[1] = Serial.read();
  byte_v[2] = Serial.read();
  byte_v[3] = Serial.read();
  target_v = *((float*)(byte_v));
 }
Serial.println(target_v);
 curr_v = get_speed(prev_pulse);
 curr_speed.data = curr_v;
 error = target_v - curr_v;
 error_sum += error;
 d_error = error - prev_error;

 motor_val = Kp*error + Ki*error_sum + Kd*d_error;

 if (target_v < 0)
 {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    motor_val = map(motor_val, -5, 5, 0, 255);
    analogWrite(EN_B, motor_val);
 }
  else 
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    motor_val = map(motor_val, -5, 5, 0, 255);
    analogWrite(EN_B, motor_val);
  }


  prev_error = error;
  time1 = micros();
  prev_pulse = pulse_count;
  nh_slave.spinOnce();
}

void EncoderInit()
{
  Direction = true;//default -> Forward  
  pinMode(encoder0pinB,INPUT);  
  attachInterrupt(0, encoder_count, CHANGE);
}

void encoder_count()
{
  int Lstate = digitalRead(encoder0pinA);
  if((encoder0PinALast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(encoder0pinB);
    if(val == LOW && Direction)
    {
      Direction = false; //Reverse
    }
    else if(val == HIGH && !Direction)
    {
      Direction = true;  //Forward
    }
  }
  encoder0PinALast = Lstate;
 
  if(!Direction)  pulse_count++;
  else  pulse_count--;
}

double get_distance(int pulse_count)
{
  double distance = double (pulse_count/PPR)*(M_PI*WHEEL_DIA);
  return distance;
}

double get_speed(int prev_pulse)
{
  time2 = micros();

  return ((double) get_distance(pulse_count - prev_pulse)/((double)(time2 - time1)/1000000.0));
    
}
