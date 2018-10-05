int photoresistorSmall = A0; // Photoresistor of the small breadboard
int photoresistorBig = A1; // Photoresistor of the big breadboard
int valueSmall = 0;
int valueBig = 0;
int i=0;
int aux = 0;
int aux1 = 0;
int motor = 10;

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

// Integer
float ki = 0.2;
float integral = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); // Enabling Serial Monitor [top-right]
  pinMode(9, OUTPUT); // DC motor controller
}

void loop() {
  int pid_sum = 0;
  
  valueSmall = analogRead(photoresistorSmall);
  valueBig = analogRead(photoresistorBig);
  //Serial.println(valueSmall);
  //Serial.println(valueBig);

  error=valueSmall-valueBig; // The error is the difference between the two photoresistors

  //proportional
  proportional=k*error;
  Serial.print("Error is:");
  Serial.println(proportional);

  //Integral & Derivative

  derivative=0;
  //while(i<8){
    valueSmall=analogRead(photoresistorSmall);
    valueBig=analogRead(photoresistorBig);
    aux=valueSmall-valueBig;
    
    ti=millis(); // Inserts time passed until this point of the loop
    
    Serial.print("ti is:");
    Serial.println(ti);
    
    //print time
    //Serial.print("Initial time:");
    //Serial.println(ti);
    
    valueSmall=analogRead(photoresistorSmall);
    valueBig=analogRead(photoresistorBig);
    aux1=valueSmall-valueBig;
    
    tf=millis();
    
    Serial.print("tf is:");
    Serial.println(tf);
    
    //print final time
    //Serial.print("Final time:");
    //Serial.println(ti);
    
    incT=(tf-ti)/1000;   // Calculate in ms
    
    derivative=derivative+(aux1-aux)/incT; 
    integral=integral+(aux1+aux)*incT/2;
    
    i++;        
  //}
  
i=0;

derivative=kd*(derivative/8);
integral=ki*integral;


Serial.print("Derivative:");
Serial.println(derivative);
Serial.print("Integral:");
Serial.println(integral);
Serial.print("Proportional:");
Serial.println(proportional);

pid_sum = derivative + integral + proportional;

Serial.print("PID value =");
Serial.println(pid_sum);

if(pid_sum > 5)
  {
    pid_sum = 5;
  }

if(pid_sum < 0)
  {
    pid_sum = 0;
  }

pid_sum = map(pid_sum,0, 5,0, 255);

Serial.print("PWM signal after mapping is:");
Serial.println(pid_sum);

analogWrite(10, pid_sum);
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
