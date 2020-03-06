#include <ros.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

std_msgs::Float32 curr_speed;
ros::Publisher chatter("chatter", &curr_speed);

double Kp = 5, Ki = 0.1, Kd = 0.15;
double error, error_sum = 0, d_error, prev_error;
double target_v = 3, curr_v;
double motor_val;
boolean Direction;
const double PPR = 224.4;
const double WHEEL_DIA = 0.11; //outer diameter of the wheel is 110 mm
int pulse_count = 0, prev_pulse = 0;
const byte encoder0pinA = 2;//A pin -> the interrupt pin 0
const byte encoder0pinB = 3;//B pin -> the digital pin 3
byte encoder0PinALast;
int EN_A = 11;      //Enable pin for first motor
int IN1 = 9;       //control pin for first motor
int IN2 = 8;       //control pin for first motor
unsigned long time1 = 0;
unsigned long time2 = 0;

//union {
//    float float_v;
//    byte byte_v[4];
//  } floatAsBytes;
  
void setup() {
  Serial.begin (9600); //Starting the serial communication at 9600 baud rate
  //Initializing the motor pins as output
  pinMode(EN_A, OUTPUT);
  pinMode(IN1, OUTPUT);  
  pinMode(IN2, OUTPUT);
  EncoderInit();//Initialize the module
//  nh.initNode();
//  nh.advertise(chatter);
  ;
  
  time1 = micros();
}

void loop() {  
// delay(50);
 curr_v = get_speed(prev_pulse);
 curr_speed.data = curr_v;
// chatter.publish(&curr_speed);
// Serial.println(curr_v);


//floatAsBytes.float_v = curr_v;

Serial.write((byte *)&curr_v, sizeof(curr_v));
byte* curr_v_byte = (byte *)&curr_v;
for (int i = 0; i < 4; i++)
{
  Serial.print(*curr_v_byte);
  Serial.println("  ");
}

delay(1000);

// /nh.spinOnce();

 error = target_v - curr_v;
 error_sum += error;
 d_error = error - prev_error;

 motor_val = Kp*error + Ki*error_sum + Kd*d_error;

 if (target_v < 0)
 {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    motor_val = map(motor_val, -5, 5, 0, 255);
    analogWrite(EN_A, motor_val);
 }
  else 
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    motor_val = map(motor_val, -5, 5, 0, 255);
    analogWrite(EN_A, motor_val);
  }


  prev_error = error;
  time1 = micros();
  prev_pulse = pulse_count;
}

void EncoderInit()
{
  Direction = true;//default -> Forward  
  pinMode(encoder0pinB,INPUT);  
  attachInterrupt(0, encoder_count, CHANGE);
}

void encoder_count()
{
  int curr_state = digitalRead(encoder0pinA);
  if((encoder0PinALast == LOW) && curr_state==HIGH)
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
  encoder0PinALast = curr_state;
 
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
