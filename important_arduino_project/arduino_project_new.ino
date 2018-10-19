int enablePin = 11;
int motorRight = 10;
int motorLeft = 9;
int photoresistors[] = {A0,A1,A2,A3};
int min_photoresistors[] = {0,0};  
int values[]= {0,0,0,0};
int r_min[] = {0,0};
int max_difference = 500; //We need to measure the exact maximum difference between two photoresistors


//Proportional Controller
float k=0.0488;
float proportional = 0;

// Derivative
float kd = 0.02;
float derivative = 0;

float ti=0;
float tf=0;
float incT=0;

int q=max_difference;
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
  
  q=getError();
  
  q_int = q; // Error of integral
  
  q_prev = q; // Previous error
  t_prev = millis();  // measure initial measurement time

}

void loop() {
  // put your main code here, to run repeatedly:
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

  pid_sum = map(abs(pid_sum),0, 50,30, 255); //Control speed, '50' needs to be changed
  analogWrite(enablePin, pid_sum); 
      if (r_min[0]<r_min[1]){          // we decide which way to rotate the DC Motor based on which value of the photoresistors is the minimum
        if(min_photoresistors[1]-min_photoresistors[0]==1){
        
          digitalWrite(motorRight, HIGH);             //Control Direction
          digitalWrite(motorLeft, LOW);
       
        }else {
        
          digitalWrite(motorRight, LOW);
          digitalWrite(motorLeft, HIGH);
        
       }
      } else {
        if(min_photoresistors[1]-min_photoresistors[0]==1){
        
          digitalWrite(motorRight, LOW);
          digitalWrite(motorLeft, HIGH);
       
        }else {
        
          digitalWrite(motorRight, HIGH);             //Control Direction
          digitalWrite(motorLeft, LOW);
      }
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

  q = max_difference;
  for (int j=0; j<4; j++){
      values[j]=analogRead(photoresistors[j]); // get the value of each photoresistor
      if (j<2){                         //we determine which value is the minimum ( between {value[0],value[2]} for j=0 and {value[1],value[3]} for j=1)
        if (values[j]<values[j+2]){
          r_min[j]=values[j];
          min_photoresistors[j]=j;     //which photoresistor is the minimum
        } else {
          r_min[j]=values[j+2];
          min_photoresistors[j]=j+2;
        }
      }
  }
  q = q - abs(r_min[1]-r_min[0]); // Error
  return q;
  
}
