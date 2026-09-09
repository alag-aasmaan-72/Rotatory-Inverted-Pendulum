#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050.h"


#define ENA 6
#define IN1 A2
#define IN2 A3

#define ENC_A  2
#define ENC_B  3

#define ENB 5
#define IN3 4
#define IN4 9

#define TRIG_PIN 10 
#define ECHO_PIN 11 

#define TRIG_PIN2 7
#define ECHO_PIN2 8


#define TRIG_PIN3 12
#define ECHO_PIN3 13 

#define MAX_DISTANCE 300



volatile long Ticks = 0;
 
MPU6050 mpu;
int left_distance ;
int front_distance ; 
int right_distance ; 

float C ; 
float correction ;
float d_time; 
float proportional ; 
float derivative ; 
float integral ; 
bool a= false   ;
bool b = false ; 
bool c = false ; 

bool j = true ;
bool i = false ; 
bool l = false ;
bool h = false ; 

volatile long  mpucurrentTime ; 
volatile long  mpuprevTime = 0 ; 

bool u = true ; 
volatile long  current_time_read ; 
volatile long  prev_time_read = 0 ; 

volatile long  current_time ; 
volatile long  prev_time = 0 ; 

volatile long  current_time2 ; 
volatile long  prev_time2 = 0 ; 

volatile long  current_time3 ; 
volatile long  prev_time3 = 0 ; 

volatile long    prev_time4 = 0 ;  ;

volatile long  current_time4 ;


volatile long  current_time5 ; 
volatile long  prev_time5 = 0 ;

unsigned long prev_int = 0 ; 
volatile long  current_time6 ; 
volatile long  prev_time6 = 0 ;



volatile unsigned long start1 = 0;
volatile unsigned long start2 = 0;
volatile unsigned long start3 = 0;

volatile unsigned long duration1 = 0;
volatile unsigned long duration2 = 0;
volatile unsigned long duration3 = 0;

volatile bool newEcho1 = false;
volatile bool newEcho2 = false;
volatile bool newEcho3 = false;

volatile byte prevPortB; 


int16_t ax, ay, az;
int16_t gx, gy, gz;
float rollAngle = 0;
float rollRate  = 0;
float accelAngle = 0;
float accelAngle2=0;

float dt = 0;
float gyroX_bias = -278.55;
float gyroZ_bias = -320.41;


unsigned long ultrasonicTimer = 0;
float prev_error_pid = 0 ; 
 
int leftDist ;
int frontDist ;
int rightDist ;

float yawRate;
float error ; 

float kp= 2  ; 
float kd = 0.7 ; 
float yaw_angle;
float x1;
float x2;
float x3;
float x4;
float theta;
float theta_dot;
float alpha;
float alpha_dot;

float ref_theta = 0 ;
float ref_theta_dot = 0;
float ref_alpha =0;
float ref_alpha_dot = 0;

unsigned long lastTrigger = 0 ;
byte sensorIndex = 0;



float k1=-1.622776601681624; //t
float k2= 131.8061689123084; //a
float k3=  -27.548644177213326; //t dot
float k4=   50.82030628242194;

float target_theta=0;

float U;


void MPU_data()
{
    unsigned long mpucurrentTime = micros();

     dt = (mpucurrentTime - mpuprevTime) / 1000000.0;

    mpuprevTime = mpucurrentTime;

// Read MPU

    mpu.getMotion6(&ax, &ay, &az,&gx, &gy, &gz);

// Roll Rate

    rollRate =(gx - gyroX_bias) / 131.0;

// alpha from acceleration
    accelAngle =atan2(ay, az) * 180.0 / PI;
// complementary filter 

    rollAngle =0.98 * (rollAngle + rollRate * dt)+0.02 * accelAngle;
    yawRate = (gz-gyroZ_bias)/131.0;
    // Serial.println(rollRate) ;
    // Serial.println(dt);
    
}

// this is drive motor function 
void driveMotor(float pwm, float u_new){

    
    
    pwm = constrain(pwm,-255,255);

    float u = constrain(u_new,-255,255);

    float final_u = map(abs(u), 0, 255, 30 ,255);

    if(pwm >= 0)
    {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        analogWrite(ENA, final_u);
    }
    else
    {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);

        analogWrite(ENA, final_u);
    }
}

// this is interrupt service routine 

void encoderISR()
{
  // Same logic as your balancing robot code

  if (digitalRead(ENC_A) > digitalRead(ENC_B))
  {
    Ticks++;
  }
  else
  {
    Ticks--;
  }
}




void mazesonver(){

if(front_distance < 9 && right_distance> 50 )  {

    BOmotor(0) ;

     Ticks = 0 ; 
    ref_theta = 0 ; 

    i = true ;  
    h = true ; 

}

}




ISR(PCINT0_vect)
{
    byte currentPortB = PINB;

    byte changed = currentPortB ^ prevPortB;

    unsigned long now = micros();

    // D8 (PB0)
    if (changed & (1 << PB0))
    {
        if (currentPortB & (1 << PB0))
            start2 = now;                 // Rising edge
        else
        {
            duration2 = now - start2;     // Falling edge
            newEcho2 = true;
        }
    }

    // D11 (PB3)
    if (changed & (1 << PB3))
    {
        if (currentPortB & (1 << PB3))
            start1 = now;
        else
        {
            duration1 = now - start1;
            newEcho1 = true;
        }
    }

    // D13 (PB5)
    if (changed & (1 << PB5))
    {
        if (currentPortB & (1 << PB5))
            start3 = now;
        else
        {
            duration3 = now - start3;
            newEcho3 = true;
        }
    }

    prevPortB = currentPortB;
}




void setup()
{
    Serial.begin(115200);
    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
     pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    analogWrite(ENA, 0);
    pinMode(ENC_A, INPUT_PULLUP);
    pinMode(ENC_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENC_A),encoderISR,RISING);
    Wire.begin();

    mpu.initialize();

    mpuprevTime = millis();
unsigned long ultrasonicTimer = millis();

    Serial.println("MPU Initialized");


 prevPortB = PINB;

// Enable pin change interrupts on PORTB
PCICR |= (1 << PCIE0);

// Enable PB0, PB3, PB5
PCMSK0 |= (1 << PCINT0);  // D8
PCMSK0 |= (1 << PCINT3);  // D11
PCMSK0 |= (1 << PCINT5);  // D13333
    delay(1000) ; 

}



void readsensor(){
     
digitalWrite(TRIG_PIN, LOW);
  current_time = micros() ;
  if(current_time - prev_time  >= 2 ) {
    prev_time = current_time ; 
    digitalWrite(TRIG_PIN, HIGH);
    a = true ; 
  }

  if(a==true ) { 

    current_time2 = micros()  ; 
    if(current_time2 - prev_time2 >= 10 )  {
       prev_time2 = current_time2 ;  

       digitalWrite(TRIG_PIN, LOW);  
       a = false ; 
    }
  }

  //long duration = pulseIn(ECHO_PIN, HIGH, 30000);

  // Calculate distance in cm
  //left_distance = duration * 0.0343 / 2.0;


 


  // Measure echo time
 // long duration2 = pulseIn(ECHO_PIN2, HIGH, 30000);

  // Calculate distance in cm
 // front_distance = duration2 * 0.0343 / 2.0;



  digitalWrite(TRIG_PIN3, LOW);
  current_time5 = micros() ;
  if(current_time5 - prev_time5  >= 2 ) {
    prev_time5 = current_time5 ; 
    digitalWrite(TRIG_PIN3, HIGH);
    c = true ; 
  }

  if(c==true ) { 

    current_time6 = micros()  ; 
    if(current_time6 - prev_time6 >= 10 )  {
       prev_time6 = current_time6 ;  

       digitalWrite(TRIG_PIN3, LOW);  
       c = false ; 
    }
  }

   digitalWrite(TRIG_PIN2, LOW);
  current_time3 = micros() ;
  if(current_time3 - prev_time3  >= 2 ) {
    prev_time3 = current_time3 ; 
    digitalWrite(TRIG_PIN2, HIGH);
    b = true ; 
  }

  if(b==true ) { 

    current_time4 = micros()  ; 
    if(current_time4 - prev_time4 >= 10 )  {
       prev_time4 = current_time4 ;  

       digitalWrite(TRIG_PIN2, LOW);  
       b = false ; 
    }
  }


  // Measure echo time
 // long duration3 = pulseIn(ECHO_PIN3, HIGH, 30000);

  // Calculate distance in cm
  //right_distance = duration3 * 0.0343 / 2.0;
  
}



 void yaw_Angle(){
    long ticksCopy;

    
    ticksCopy = Ticks;

    float wheel_angle= ticksCopy*2;
    float arc = (wheel_angle*2*PI*3)/360 ;
    yaw_angle  = ((arc/18.0)*(180/PI));
    // Serial.println(yaw_angle);
  
}
void BOmotor(float pwm)
{
    pwm = constrain(pwm, -255, 255);

    // Deadband compensation
    if(pwm > 0)
        pwm = map(pwm, 0, 255, 0, 255);
    else if(pwm < 0)
        pwm = -map(abs(pwm), 0, 255, 0, 255);

    if(pwm >= 0)
    {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);

        analogWrite(ENB, pwm);
    }
    else
    {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);

        analogWrite(ENB, abs(pwm));
    }

}



void centerline(){

    if(j==true ){
    if(right_distance >30){

         C = -1 ; 
         return ; 
    }

proportional = left_distance - right_distance ; 
error = proportional ; 

derivative = (error - prev_error_pid) /0.04  ; 
prev_error_pid = error ; 

integral = integral + error*d_time ; 
integral = constrain(integral , -15 , 15 ) ;

//C = 1*proportional+0.001*derivative+ 0.001*integral;
C = 1*proportional ;
C = constrain(C , -10, 10 ) ;

    }

}

float targetTheta = 0;
 void loop()
{
    // prevTime = millis();
    MPU_data();
    yaw_Angle(); 

    theta = yaw_angle;
    theta_dot= yawRate;
    alpha =  -(rollAngle);
    alpha_dot = -(rollRate);

   
    x1=(ref_theta - theta) ;  
    x2=ref_alpha-alpha;
    x3=ref_theta_dot-theta_dot;
    x4=ref_alpha_dot-alpha_dot;
    U = 0.25*(-k1*x1+k2*x2 - k3*x3 + k4*x4);
  
    driveMotor(U,U);
   
   //docentering() ;
if(i==false)
{   if(C>7 ||  C<-7) {
     BOmotor(0) ;
}


else{
    BOmotor(110) ;
}
    
}

if(h == false ) {

    mazesonver() ; 

}


//Serial.println(ref_alpha);

current_time_read = millis() ; 
if(current_time_read - prev_time_read  >= 10) {

    prev_time_read = current_time_read ; 
    readsensor() ;

}


if(newEcho1)
{
    noInterrupts();
    unsigned long d = duration1;
    newEcho1 = false;
    interrupts();

    left_distance = d * 0.0343 / 2.0;
}

if(newEcho2)
{
    noInterrupts();
    unsigned long d = duration2;
    newEcho2 = false;
    interrupts();

    front_distance = d * 0.0343 / 2.0;
}

if(newEcho3)
{
    noInterrupts();
    unsigned long d = duration3;
    newEcho3 = false;
    interrupts();

    right_distance = d * 0.0343 / 2.0;
}


if(i==true)
{


 j = false ; 
    ref_alpha = -6 ; 

    if(theta < -83) {

        ref_alpha = 0 ; 
        Ticks  = 0 ;  
        ref_theta = 0 ;


        j = true ; 
        u = false  ; 
        
        i = false ;
        
         

}
}
centerline() ;

if(u==false) {
if(millis() - prev_int  >= 3000) {

    prev_int = millis() ;
    h = false ; 
    u = true ;
}
}
//Serial.println( ref_alpha) ;

/*
Serial.print(ref_alpha);
Serial.print("     ") ; 
Serial.print( left_distance ) ;
Serial.print("     ") ; 
Serial.print( front_distance) ;
Serial.print("     ") ; 
Serial.print( right_distance) ;
Serial.print("     ") ; 
Serial.print( theta ) ;
Serial.print("     ") ; 
Serial.println( ref_theta ) ;
*/
if(j==true){
ref_alpha= C ; 
}
Serial.println(ref_alpha);


/*

Serial.print( left_distance ) ;
Serial.print("     ") ; 
Serial.print( front_distance) ;
Serial.print("     ") ; 
Serial.println( right_distance) ;

*/
//Serial.print("     ") ; 
//Serial.println(theta) ; 
//Serial.print("     ") ; 
//Serial.println(x1) ; 

    //BOmotor(60);  
}


