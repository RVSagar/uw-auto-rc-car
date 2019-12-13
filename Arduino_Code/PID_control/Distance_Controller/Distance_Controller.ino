double Kp = 0.6, Ki = 0, Kd = 1;
double error_sum, d_error, last_error;
double error, target_dis = 10, curr_dis;
double motor_val;
const double PPR = 224.4;
const double WHEEL_DIA = 0.11; //outer diameter of the wheel is 110 mm
int pulse_count = 0;
boolean Direction;
const byte encoder0pinA = 2;//A pin -> the interrupt pin 0
const byte encoder0pinB = 3;//B pin -> the digital pin 3
byte encoder0PinALast;
int EN_A = 11;      //Enable pin for first motor
int IN1 = 9;       //control pin for first motor
int IN2 = 8;       //control pin for first motor

void setup() {
      Serial.begin (9600); //Starting the serial communication at 9600 baud rate
      //Initializing the motor pins as output
      pinMode(EN_A, OUTPUT);
      pinMode(IN1, OUTPUT);  
      pinMode(IN2, OUTPUT);
      EncoderInit();//Initialize the module
}

void loop() {
  Serial.print("  Distance:");
  Serial.println(curr_dis);
  Serial.print("  ");
  Serial.print(motor_val);
  curr_dis = get_distance(pulse_count);
  error = target_dis - curr_dis;
  error_sum += error;
  d_error = error - last_error; //d_error = (error-last_error)/loop_time;  dont have to consider time. change kd
  
  
  motor_val = Kp*error + Ki*error_sum + Kd*d_error;

  if (motor_val >= 0)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    motor_val = map(motor_val, -20, 20, 0, 255);
    analogWrite(EN_A, motor_val);
  }
  else
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    motor_val = map(motor_val, -20, 20, 0, 255);
    analogWrite(EN_A, motor_val);
  }

  if (abs(error) < 0.1)
  {
    analogWrite(EN_A, LOW);
  }
  
  last_error = error;
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

double get_speed(double time_elapsed)
{
  return double (get_distance(pulse_count)/time_elapsed);
}
