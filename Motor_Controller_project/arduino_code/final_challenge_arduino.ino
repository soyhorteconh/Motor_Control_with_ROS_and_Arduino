
#include <ros.h>
#include <std_msgs/Int32.h>
#include <final_challenge/motor_msg.h>
#include <final_challenge/motor_output.h>
#include <LiquidCrystal.h>

//Node
ros::NodeHandle motor;

#define encoder0PinA  2
#define encoder0PinB  4

volatile long encoder0Pos = 0;
long newposition;
long oldposition = 0;
unsigned long newtime;
unsigned long oldtime = 0;
double vel;

//Motor pin
int enable = 6;
int entrada1 = 7;
int entrada2 = 8;

double vec[5] = {0,0,0,0,0};

final_challenge::motor_output motor_output_msg;

//LCD Pin
const int rs = 12, en = 11, d4 = 5, d5 = 10, d6 = 3, d7 = 9;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//Callback
void callback(const final_challenge::motor_msg& msg){
  double u_val = 255.0 * msg.pwm;
  
  if ((String) msg.direction == "forward" ){
    digitalWrite(entrada1, HIGH);
    digitalWrite(entrada2, LOW);
    analogWrite(enable, u_val);
    motor_output_msg.direction = "forward";
    
    
  } else if ((String) msg.direction == "backward"){
    digitalWrite(entrada1, LOW);
    digitalWrite(entrada2, HIGH);
    analogWrite(enable, u_val);
    motor_output_msg.direction = "backward";
  }
  
}

//Topic
ros::Subscriber<final_challenge::motor_msg> sub("motor_input", &callback);

ros::Publisher pub_msg("motor_output", &motor_output_msg);

double calcMean(double newVal){
  double sum = newVal;
  double temp, newA = newVal;
  for(int i = 0; i < 4; i++){
    sum += vec[i];
    temp = vec[i];
    vec[i] = newA;
    newA = temp;
    //vec++;
  }
  vec[4] = newA;
  
  return (sum/5);
}

void setup()
{
  //ros
  motor.initNode();
  motor.subscribe(sub);
  motor.advertise(pub_msg);

  //motor
  pinMode(encoder0PinA, INPUT);
  digitalWrite(encoder0PinA, HIGH);       // turn on pullup resistor
  pinMode(encoder0PinB, INPUT);
  digitalWrite(encoder0PinB, HIGH);       // turn on pullup resistor
  attachInterrupt(0, doEncoder, RISING);  // encoDER ON PIN 2

  //motor
  pinMode(enable, OUTPUT);
  pinMode(entrada1, OUTPUT);
  pinMode(entrada2, OUTPUT);
  
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  // Print a message to the LCD.
  lcd.print("DC MOTOR");
}

void loop()
{
  motor.spinOnce();
  delay(1);
    
  newposition = encoder0Pos;
  newtime = millis();

  vel = 2*3.1415*(newposition-oldposition)*1000/((newtime-oldtime)*350);
  vel = calcMean(vel);
   
  oldposition = newposition;
  oldtime = newtime;

  motor_output_msg.vel = vel;
  motor_output_msg.flag = 1;

  pub_msg.publish(&motor_output_msg);

  delay(15);
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print("Vel: ");
  lcd.print(vel);
  lcd.print(" rad/s");
}

void doEncoder()
{
  if (digitalRead(encoder0PinA) != digitalRead(encoder0PinB)) {
    encoder0Pos++;
  } else {
    encoder0Pos--;
  }
}
