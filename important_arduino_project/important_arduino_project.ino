int photoresistorSmall = A2; // Photoresistor of the small breadboard
int photoresistorBig = A3; // Photoresistor of the big breadboard
int valueSmall = 0;
int valueBig = 0;
int i=0;
int enablePin = 11;
int motorRight = 10;
int motorLeft = 9;


float error=0;

//Proportional Controller
float k=0.0488;
float proportional = 0;

// Derivative
float kd = 0.02;
float derivative = 0;

float ti=0;
float tf=0;
float incT=0;

int q=0;
int q_int=0;
int q_prev=0;
int t_prev=0;
int t_new = 0;
int dt = 0;

// Integer
float ki = 0.2;
float integral = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Enabling Serial Monitor [top-right]
  pinMode(enablePin, OUTPUT);          // Control motor speed 
  pinMode(motorRight, OUTPUT);         // Pins for 
  pinMode(motorLeft, OUTPUT);          // direction 
  
  valueSmall=analogRead(photoresistorSmall); // get initial values
  valueBig=analogRead(photoresistorBig);
  
  q = valueSmall-valueBig; // Initial Error
  q_int = q; // Error of integral
  
  q_prev = q; // Previous error
  t_prev = millis();  // measure initial measurement time
 
}

void loop() {
  int pid_sum = 0; // Sum of Proportional, Integral and Derivative controller

  q = getError(); // Function that calculates the new error
  t_new = millis();
 
  //Serial.print("q = ");
  //Serial.println(q);

  //proportional
  proportional=k*q;

  //Integral & Derivative

  dt = (t_new - t_prev)/1000; // Difference between the 2 measurements

  derivative = (q-q_prev)/dt; 
  
  q_prev = q;
  t_prev = t_new;

  q_int = q_int + q * dt;
    
    //incT=(tf-ti)/1000;   // Calculate in ms

  derivative=kd*derivative;
  integral=ki * q_int;

  

  pid_sum = integral + proportional;

  Serial.print("PID value =");
  Serial.println(pid_sum);
  
  
       if(pid_sum > 0){
        
      pid_sum = map(pid_sum,0, 50,30, 255);
      analogWrite(enablePin, pid_sum);            //Control speed
      digitalWrite(motorRight, HIGH);             //Control Direction
      digitalWrite(motorLeft, LOW);
       
       }else if(pid_sum < 0){
        
      pid_sum = map(-pid_sum,0, 50,30, 255);
      analogWrite(enablePin, pid_sum);
      digitalWrite(motorRight, LOW);
      digitalWrite(motorLeft, HIGH);
        
       }
  
  
  Serial.print("PWM signal after mapping is:");
  Serial.println(pid_sum);
  
  //analogWrite(10, pid_sum);
  Serial.println("----------------------------");


/* 
 * We are using only one transistor, so DC motor can turn into only one direction. In order to make the 
 * motor to rotate to the opposite direction we have to connect a second digiatal pin to dc motor with 
 * another transistor as well and connect the transistor reverse-biased. So, if we have a value that is 
 * below 0 in pid_sum then we analogWrite(pinA, 0) to switch off the first pin (not to rotate in the
 * previous direction and analogWrite(pinB, -pid_sum). Minus is very important because PWM cannot interpret
 * negative values.
 * 
 */

}

int getError(){

  
  int valueSmall=analogRead(photoresistorSmall);
  int valueBig=analogRead(photoresistorBig);
  
  return valueSmall-valueBig;
  
}
