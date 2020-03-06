#include <PinChangeInt.h>

#include <ros.h>
#include <std_msgs/Float32.h>


ros::NodeHandle nh;

std_msgs::Float32 curr_speed;
ros::Publisher chatter("chatter", &curr_speed);

double Kp = 6, Ki = 0, Kd = 0; //Kd = 0.15
double error_0, error_sum_0 = 0, d_error_0, prev_error_0, error_1, error_sum_1 = 0, d_error_1, prev_error_1;
double error_2, error_sum_2 = 0, d_error_2, prev_error_2, error_3, error_sum_3 = 0, d_error_3, prev_error_3;
double target_v = 3, curr_v_0, curr_v_1, curr_v_2, curr_v_3;
double motor_val_0, motor_val_1, motor_val_2, motor_val_3;
boolean Direction0, Direction1;
const double PPR = 224.4;
const double WHEEL_DIA = 0.11; //outer diameter of the wheel is 110 mm
int pulse_count_0 = 0, prev_pulse_0 = 0, pulse_count_1 = 0, prev_pulse_1 = 0, pulse_count_2 = 0, prev_pulse_2 = 0, pulse_count_3 = 0, prev_pulse_3 = 0;
const byte encoder0pinA = 2;//A pin -> the interrupt pin 0
const byte encoder0pinB = A0;//B pin -> the digital pin 3
const byte encoder1pinA = 9;//interrupt pin (on digital pin 4) for slave encoder
const byte encoder1pinB = 10;//direction pin (on digital pin 5) for slave encoder
const byte encoder2pinA = A4;//A pin -> the interrupt pin 0
const byte encoder2pinB = A5;//B pin -> the digital pin 3
const byte encoder3pinA = A2;//interrupt pin (on digital pin 4) for slave encoder
const byte encoder3pinB = A3;//direction pin (on digital pin 5) for slave encoder
byte encoder0PinALast, encoder1PinALast, encoder2PinALast, encoder3PinALast;
const int E0 = 3;//speed output for motor 0 (Motor 1 on the driver board)
const int E1 = 11;//speed output for motor 1 (Motor 2 on the driver board)
const int E2 = 5;//speed output for motor 0 (Motor 1 on the driver board)
const int E3 = 6;//speed output for motor 1 (Motor 2 on the driver board)

const int M0 = 4; //direction output for motor 0 (motor 1 on the driver board)
const int M1 = 12; //direction output for motor 1 (motor 2 on the driver board)
const int M2 = 8; //direction output for motor 0 (motor 1 on the driver board)
const int M3 = 7; //direction output for motor 1 (motor 2 on the driver board)

unsigned long time1 = 0;
unsigned long time2 = 0;

//union {
//    float float_v;
//    byte byte_v[4];
//  } floatAsBytes;
  
void setup() {
  Serial.begin (9600); //Starting the serial communication at 9600 baud rate
  //Initializing the motor pins as output
  pinMode(E0, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(E2, OUTPUT);
  pinMode(E3, OUTPUT);  
  pinMode(M0, OUTPUT);
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(M3, OUTPUT);
  EncoderInit();//Initialize the module
  nh.initNode();
  nh.advertise(chatter);
  ;
  
  time1 = micros();
}

void loop() {  
 delay(50);

 curr_v_0 = get_speed(prev_pulse_0, pulse_count_0);
 curr_v_1 = get_speed(prev_pulse_1, pulse_count_1);
 curr_v_2 = get_speed(prev_pulse_2, pulse_count_2);
 curr_v_3 = get_speed(prev_pulse_3, pulse_count_3);
 
 //-------------ROS PUBLISHER--------
 curr_speed.data = curr_v_1; 
 chatter.publish(&curr_speed);

 nh.spinOnce();
 //----------------------------------
 //MOTOR 0
 error_0 = target_v - curr_v_0;
 error_sum_0 += error_0;
 d_error_0 = error_0 - prev_error_0;
 motor_val_0 = Kp*error_0 + Ki*error_sum_0 + Kd*d_error_0;

 //MOTOR 1
 error_1 = target_v - curr_v_1;
 error_sum_1 += error_1;
 d_error_1 = error_1 - prev_error_1;
 motor_val_1 = Kp*error_1 + Ki*error_sum_1 + Kd*d_error_1;

 //MOTOR 2
 error_2 = target_v - curr_v_2;
 error_sum_2 += error_2;
 d_error_2 = error_2 - prev_error_2;
 motor_val_2 = Kp*error_2 + Ki*error_sum_2 + Kd*d_error_2;

 //MOTOR 3
 error_3 = target_v - curr_v_3;
 error_sum_3 += error_3;
 d_error_3 = error_3 - prev_error_3;
 motor_val_3 = Kp*error_3 + Ki*error_sum_3 + Kd*d_error_3;


 Serial.print("motor 0: ");
 Serial.print(curr_v_0);
 Serial.print(" motor 1: ");
 Serial.print(curr_v_1);
 Serial.print(" motor 2: ");
 Serial.print(curr_v_2);
 Serial.print(" motor 3: ");
 Serial.print(curr_v_3);
 Serial.println();

 
 if (target_v < 0)
 {
    digitalWrite(M0, LOW);
    digitalWrite(M1, HIGH);
    digitalWrite(M2, LOW);
    digitalWrite(M3, HIGH);
    motor_val_0 = map(motor_val_0, -5, 5, 0, 255);
    analogWrite(E0, motor_val_0);

    motor_val_1 = map(motor_val_1, -5, 5, 0, 255);
    analogWrite(E1, motor_val_1);

    motor_val_2 = map(motor_val_2, -5, 5, 0, 255);
    analogWrite(E2, motor_val_2);

    motor_val_3 = map(motor_val_3, -5, 5, 0, 255);
    analogWrite(E3, motor_val_3);
 }
  else 
  {
    digitalWrite(M0, HIGH);
    digitalWrite(M1, LOW);
    digitalWrite(M2, HIGH);
    digitalWrite(M3, LOW);
    motor_val_0 = map(motor_val_0, -5, 5, 0, 255);
    analogWrite(E0, motor_val_0);

    motor_val_1 = map(motor_val_1, -5, 5, 0, 255);
    analogWrite(E1, motor_val_1);

    motor_val_2 = map(motor_val_2, -5, 5, 0, 255);
    analogWrite(E2, motor_val_2);

    motor_val_3 = map(motor_val_3, -5, 5, 0, 255);
    analogWrite(E3, motor_val_3);
  }

  prev_error_0 = error_0;
  prev_error_1 = error_1;
  prev_error_2 = error_2;
  prev_error_3 = error_3;
  time1 = micros();
  prev_pulse_0 = pulse_count_0;
  prev_pulse_1 = pulse_count_1;
  prev_pulse_2 = pulse_count_2;
  prev_pulse_3 = pulse_count_3;
}

//for (int i = 0; i <= x; i++) {
//  Serial.println(buffer[i]);
// }
 
void EncoderInit()
{
  Direction0 = true;//default -> Forward  
  Direction1 = true;
  pinMode(encoder0pinB, INPUT);
  pinMode(encoder1pinB, INPUT);
  pinMode(encoder2pinB, INPUT);
  pinMode(encoder3pinB, INPUT);
  attachInterrupt(0, interrupt_func0, CHANGE);
  attachPinChangeInterrupt(encoder1pinA, interrupt_func1, CHANGE);
  attachPinChangeInterrupt(encoder2pinA, interrupt_func2, CHANGE);
  attachPinChangeInterrupt(encoder3pinA, interrupt_func3, CHANGE);
}

void interrupt_func0() {
  encoder_count(pulse_count_0, encoder0pinA, encoder0PinALast, encoder0pinB);
}
void interrupt_func1() {
  encoder_count(pulse_count_1, encoder1pinA, encoder1PinALast, encoder1pinB);
}
void interrupt_func2() {
  encoder_count(pulse_count_2, encoder2pinA, encoder2PinALast, encoder2pinB);
}
void interrupt_func3() {
  encoder_count(pulse_count_3, encoder3pinA, encoder3PinALast, encoder3pinB);
}

void encoder_count(int &pulse_count_, byte encoder_pin_A_, byte encoder_pinA_last_, byte encoder_pin_B_)
{
  int curr_state0 = digitalRead(encoder_pin_A_);
  if((encoder_pinA_last_ == LOW) && curr_state0==HIGH)
  {
    int val = digitalRead(encoder_pin_B_);
    if(val == LOW && Direction0)
    {
      Direction0 = false; //Reverse
    }
    else if(val == HIGH && !Direction0)
    {
      Direction0 = true;  //Forward
    }
  }
  encoder_pinA_last_ = curr_state0;
 
  if(!Direction0)  pulse_count_++;
  else  pulse_count_--;   
}

double get_distance(int pulse_count_)
{
  double distance = double (pulse_count_/PPR)*(M_PI*WHEEL_DIA);
  return distance;
}

double get_speed(int prev_pulse_, int pulse_count_)
{
  time2 = micros();

  return ((double) get_distance(pulse_count_ - prev_pulse_)/((double)(time2 - time1)/1000000.0));
    
}
