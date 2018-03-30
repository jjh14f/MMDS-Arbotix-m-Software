#define PAN 0        //Pan motor ID 0
#define TILT 1       //Tilt motor ID 1
#define FIRE_PIN1 D2  //Pin D0 set as firing pin
#define FIRE_PIN2 41  //Pin D1 set as firing pin

#define TILT_RIGHT_LIMIT 1365      //Safety limits on Tilt motor
#define TILT_LEFT_LIMIT 2048

#define PAN_LEFT_LIMIT 2690      //Safety limits on Pan motor
#define PAN_RIGHT_LIMIT 1365

#define DEFAULT_PAN 2048          //Default positions motors will return to
#define DEFAULT_TILT 1706         //after firing or if communication is lost

#include <mx64.h>                //Dynamixel mx64 library
#include <BioloidController.h>   //Bioloid library
#include "poses.h"               //Aiming library essentially
#include <math.h>

BioloidController bioloid = BioloidController(1000000);    //Bioloid object with baud rate 1Mbps

int pan;                         //Pan position (motor degrees)
int tilt;                        //Tilt position (motor degrees)

void setup(){
  pinMode(0,OUTPUT);            //setup user LED
  pinMode(2,OUTPUT);     //set pin D0 as output pin to send signal
  pinMode(FIRE_PIN2,OUTPUT);     //set pin D1 as output pin to send signal
  Serial.begin(9600);           //set up serial comm @ 9600 bps
  SetPosition(PAN,DEFAULT_PAN);
  SetPosition(TILT,DEFAULT_TILT);
}

void loop(){ 
  
  double x, y ,z, t, r, theta, phi, traveltime;
  double V = 30; //Speed in Meters/Second
  double Vsqr = 900; //Square of speed
  double Vfour = 810000; //Fourth power of speed
  double g = 9.81; //Acceleration due to gravity in m/s^2
  double pi = 3.14159;
  
   if(Serial.available() > 0){                   //sends data only when data is received 
    Serial.println("***********************");
    x = Serial.parseFloat();             //sets input as first valid float from serial buffer                                         
    y = Serial.parseFloat();            //sets input as second valid float from serial buffer
    z = Serial.parseFloat();            //sets input as third valid float from serial buffer

    //cartesian to spherical transform
    theta = atan2(y,x)*(180/pi);
    r = sqrt(pow(x,2) + pow(y,2) + pow(z,2));
    phi = atan((Vsqr - sqrt(Vfour - g*(g*pow(r,2) + 2*Vsqr*z)))/(g*r))*(180/pi);
    
    //spherical degree to motor degree transform
    pan = (theta + 180)*11.364;
    tilt = (phi + 180)*11.364 - 341;
      
    Serial.print("Pan & Tilt: ");
    Serial.println(pan);
    Serial.println(tilt);
      
    
    BOUNDARIES();                          //enforce boundaries
    
    Serial.print("x y z: ");
    Serial.println(x);
    Serial.println(y);
    Serial.println(z);
    Serial.print("r: ");
    Serial.println(r);
    Serial.print("theta: ");
    Serial.println(theta);
    Serial.print("phi: ");
    Serial.println(phi);
    
    Serial.print("Pan: "); 
    Serial.println(pan); 
    Serial.print("Tilt: ");
    Serial.println(tilt);
    Serial.println("Travel time: ");
    Serial.print(traveltime);
    while(Serial.available()==0) {}
    AIM();
    FIRE();
   }
delay(2000); 
//RETURN();
 //Serial.end();
}

//function that enforces the limits for the servos
void BOUNDARIES(){
  //Enforce the limits for Tilt servo
  if (tilt < TILT_RIGHT_LIMIT) {tilt = TILT_RIGHT_LIMIT; }  
  if (tilt > TILT_LEFT_LIMIT){tilt = TILT_LEFT_LIMIT;}

  //Enforce the limits for Pan servo
  if (pan < PAN_RIGHT_LIMIT){pan = PAN_RIGHT_LIMIT;}  
  if (pan > PAN_LEFT_LIMIT){pan = PAN_LEFT_LIMIT;}
}

void AIM(){
   SetPosition(PAN,pan);
   SetPosition(TILT,tilt);
}

void FIRE(){
  digitalWrite(2,HIGH);
  digitalWrite(FIRE_PIN2, HIGH);
  delay(500);
  digitalWrite(2,LOW);
  digitalWrite(FIRE_PIN2, LOW);
  delay(500);
}

//Function that returns servos to default position
void RETURN(){
  SetPosition(PAN,DEFAULT_PAN);
  SetPosition(TILT,DEFAULT_TILT);
}
