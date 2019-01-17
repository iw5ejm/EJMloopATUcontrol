/*
   Loop controller by IW5EJMarco
   A stepper motor for  loop antenna tuning
   Credits to on7eq for the tuning algorithm
   http://www.qsl.net/on7eq/projects/arduino_atu.htm
   
   reference:
   http://www.grvdc.eu
   http://www.arduino.cc/en/Reference/Stepper
   This  code is in the public domain.
*/

#include <Stepper.h>

// change this to the number of steps of your motor
#define STEPS 2048

// change this to adapt to your gear reduction coupling factor
#define Frid 1 

// change this to choose High and low motor speeds
#define Hspeed 10 //High speed (RPM)
#define Lspeed 3 // Low speed (RPM)

// defining tuning steps
#define Cstep 32 // Coarse step, adjust for 10° 
#define Fstep 4 // Fine step,  adjust for 1°

#define SWR11 20 //level for 1:1 SWR read on RefPin (A0) pin

// Define pin for user input
#define TunePin 2 // this button start automatic tuning process
#define UpPin 3  // this button move the capacitor to the right
#define DwnPin 4 // this button move the capacitor to the left
#define TuneLedPin 5 //pin for led "tuning in progress" indicator
#define SWRokLedPin 6 //
#define RefPin A0  // reflected power input

// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(STEPS, 8, 10, 9, 11);


int bestRefl = 1023;
int Swr_refl_cur = 0;
int Swr_refl_prev = 0;
int refl = 0;
byte  TuneRequestPin_status = (1);                                // High level is idle
long  TuneRequestPinbuttonTime = 0;
int capPos = 0;
int bestCapPos = 0;
int HighCapPos = 0;
int LowCapPos = 0;

void setup() {

  pinMode(RefPin, INPUT);
  pinMode(TunePin, INPUT_PULLUP);
  pinMode(UpPin, INPUT_PULLUP);
  pinMode(DwnPin, INPUT_PULLUP);
  pinMode(TuneLedPin, OUTPUT);
  pinMode(SWRokLedPin, OUTPUT);
  pinMode(FridPin, INPUT);

  digitalWrite(TuneLedPin, HIGH);
  digitalWrite(SWRokLedPin, HIGH);

  delay(500); // Test led

  digitalWrite(TuneLedPin, LOW);
  digitalWrite(SWRokLedPin, LOW);
  
  Serial.begin(9600);
  Serial.println("IW5EJM ATU loop controller v2.1");
  delay(2000);
   
  if (!digitalRead(UpPin)&&!digitalRead(DwnPin)) //press up&down button at boot to test reflected power reading
  {
   Serial.println("Entering loop for reflected power measuring");  
   while(1) {Serial.println(analogRead(RefPin)); }
  }

}

void loop() {

waitbutton:

// To verify reflected power reading:
//Serial.print("Reflected: "); Serial.println(analogRead(RefPin));
   
  if (!digitalRead(UpPin))  {
    Serial.println("button UP pressed");    
    stepper.setSpeed(Lspeed);  // manual tuning requested
    stepper.step(Fstep);
  //  if (analogRead(RefPin)<<SWR11) digitalWrite(SWRokLedPin, HIGH); else   digitalWrite(SWRokLedPin, LOW);
  }

  
  if (!digitalRead(DwnPin)) {
    Serial.println("button DOWN pressed");
    stepper.setSpeed(Lspeed);
    stepper.step(-Fstep);
  //  if (analogRead(RefPin)<<SWR11) digitalWrite(SWRokLedPin, HIGH); else   digitalWrite(SWRokLedPin, LOW);
  }

  if (digitalRead(TunePin)) goto waitbutton;


  //Start tuning process
  Serial.println("Automatic tuning requested");

  digitalWrite(TuneLedPin, HIGH);
  digitalWrite(SWRokLedPin, LOW);
  stepper.setSpeed(Hspeed);
  
  ///// find best reflection  /////

  /// coarse

  for (capPos = 0; capPos <= Frid*STEPS; capPos += Cstep) {

    stepper.step(Cstep);
    //Swr_refl_prev = Swr_refl_cur;
    Swr_refl_cur = analogRead(RefPin);
    refl = Swr_refl_cur; //(Swr_refl_cur + Swr_refl_prev)/2;
    Serial.print("COARSE tuning, Reflected: ");  Serial.print(refl);
    Serial.print(" Pos: ");  Serial.println(capPos);

    
    if (refl > 999) refl = 999;
    if (refl < bestRefl) {
      bestRefl = refl;
      bestCapPos = capPos;

    }
  }
  
Serial.println("COARSE TUNING ENDED");
Serial.print("Best refl: "); Serial.println(refl);
Serial.print("Best cap pos: "); Serial.println(bestCapPos);
  
HighCapPos = 3 *Cstep;
LowCapPos = bestCapPos - 3 *Cstep;
  
stepper.step(-(Frid*STEPS - LowCapPos));      // turn capacitor to start position

Serial.print("Cap prepared for fine tuning, actual reflected: "); Serial.println(analogRead(RefPin));
  
  /// fine
  stepper.setSpeed(Lspeed);

  bestRefl = 1023;
  //Swr_refl_cur = 0;
 // Swr_refl_prev = 0;
  capPos = 0;

  for (capPos = 0; capPos < (HighCapPos); capPos += Fstep) {

    stepper.step(Fstep);
    //Swr_refl_prev = Swr_refl_cur;
    Swr_refl_cur = analogRead(RefPin);
    refl = Swr_refl_cur; //(Swr_refl_cur + Swr_refl_prev)/2;
    Serial.print("FINE tuning, Reflected: ");  Serial.print(refl); 
    Serial.print(" Pos: ");  Serial.println(capPos);

    if (refl > 999) refl = 999;
    if (refl < bestRefl) {
      bestRefl = refl;
      bestCapPos = capPos;
    }
  }

  stepper.step(-(HighCapPos - bestCapPos));   // turn capacitor to best position


  digitalWrite(TuneLedPin, LOW);
  Serial.print("Tuning process ended, Reflected: "); Serial.println(refl);
  Serial.print("Best cap pos: "); Serial.print(bestCapPos); Serial.print("/"); Serial.println(HighCapPos);
  if (bestRefl<SWR11) {
          digitalWrite(SWRokLedPin, HIGH);
          delay(500);
          digitalWrite(SWRokLedPin, LOW);
          delay(200);
          digitalWrite(SWRokLedPin, HIGH);
          delay(200);          
          digitalWrite(SWRokLedPin, LOW);
          delay(200);
          digitalWrite(SWRokLedPin, HIGH);
          delay(200);
          digitalWrite(SWRokLedPin, LOW);
          delay(200);
          digitalWrite(SWRokLedPin, HIGH);
          delay(200);
          digitalWrite(SWRokLedPin, LOW);
          }
}
