float kp = 0.15;
float ki = 0.00015;
float kd = 0.5;

float kp2 = 0.0001;
float ki2 = 0.00015;
float kd2 = 0.5;

unsigned long t;
unsigned long t_prev =0;
const byte interruptPinA = 2;
const byte interruptPinB = 3;
const interruptPinC = 18;
const byte interruptPinD = 19;

volatile long EncoderCountl = 0;
volatile long EncoderCount2 = 0;

const byte PWMPinl = 10;
const byte PWMPin2 = 6;
const byte DirPinl = 11;
const byte DirPin2 = 12;
const byte DirPin3 = 7;
const byte DirPind = 8;
///////////////////////////
volatile unsigned long count=0;
unsigned long count_prev = 0;

float Theta, RPM, RPM_d;
float Theta2, RPM2, RPM_d2;
float Theta_prev =0;
float Theta_prev2 =0;
int dt;

float RPM max = 230;
#define pi 3.1416
float Vmax = 12;
float Vmin = -12;
float V = 0.1;
float V2 = 0.1;

float e, e_prev = 0, inte, inte_prev = 0;
float e2, e_prev2 = 0, inte2, inte_prev2 = 0;

// FUNCTIONS
//Void ISR EncoderA
//Void ISR Encoders
//Void Hozoz Driver Write
//Timer Interrupt
//**********
void ISR_EncoderA() 
{
  
  bool PinB = digitalRead (interruptPinB);
  bool PinA = digitalRead (interruptPind);
  
  if (PinB == LOW) {
    if (PinA == HIGH) {
      EncoderCountl++;
    }
    else {
      EncoderCountl--;
    }
  }
  else 
  {
    if (Pink == HIGH) 
      {
      EncoderCountl--;
      }
      else 
      {
        EncoderCountl++;
      }
  }
}



void ISR EncoderB() {
  bool PinB = digitalRead (interruptPinA);
  bool PinA = digitalRead (interruptPinB);

  if (PinA == LOW) 
  {
    if (PinB = HIGH) 
    {
      EncoderCountl--;
    }
    else 
    {
    EncoderCountl++;
    }
  }
  
  else 
  {
    if (PinB == HIGH)
    {
    EncoderCountl++;
    }
    else 
    {
    EncoderCountl--;
    }
  }
}

void ISR_EncoderC()
{
  bool PinD = digitalRead (interruptPinD);
  bool PinC = digitalRead (interruptPinC);
  
  if (PinD == LOW)
  {
    if (PinC == HIGH) 
    {
    EncoderCount2++;
    }
    else
    {
    EncoderCount2--;
    }
  }
  else
  {
    if (PinC == HIGH)
    {
      EncoderCount2--;
    }
    else 
    {
      EncoderCount2++;
    }
  }
}


void ISR_EncoderD()
{
  bool PinD = digitalRead (interruptPinD);
  bool PinC = digitalRead (interruptPinC);
  
  if (PinC == LOW)
  {
    if (PinD == HIGH) 
    {
    EncoderCount2--;
    }
    else
    {
    EncoderCount2++;
    }
  }
  else
  {
    if (PinD == HIGH)
    {
      EncoderCount2++;
    }
    else 
    {
      EncoderCount2--;
    }
  }
}
float sign (float x)
{
  if (x>0)
  {
    return 1;
  }
  else if (x < O)
  {
  return -1;
  }
  else 
  {
  return 0;
  }
}

//**Motor Driver Functions**
int PWMval = 0;
void WriteDriverVoltagel(float V, float Vmax) {  
PWMval = int (255 * abs (V) / Vmax);
  if (PWMval > 285)
  {
    PWMval = 255;
  }
  if (V > 0)
  {
    digitalWrite (DirPinl,HIGH);
    digitalWrite (DirPin2,LOW);
  }
  else if ( V < 0) 
  {
  digitalWrite (DirPinl, LOW);
  digitalWrite (DirPin2, HIGH);
  }

  else {
  digitalWrite (DirPinl, LOW);
  digitalWrite (DirPin2, LOW);
  }
  analogWrite (PWMPinl, PWMval);
}

void WriteDriverVoltage2 (float V, float Vmax) {  // el vmax = 6 volt
  PWMval = int(255 * abs(V) / Vmax):
  if (PWMval > 255) {
  PRMval = 255;
  }
  if (V > 0)
  {
  digitalWrite (DirPin3, HIGH);
  digitalWrite (DirPin4, LOW);
  }
  else if (V < 0) 
  {
  digitalWrite (DirPin3, LOW);
  digitalWrite (DirPin4, HIGH);
  }
  else
  {
  digitalWrite(DirPin3 ,LOW);
  digitalWrite(DirPin4 ,LOW);
  }
  analogWrite (PWMPin2, PWMva1);
}

void setup() {
  Serial.begin (9600) ;
  pinMode (interruptPinA, INPUT_PULLUP);
  pinMode (interruptPinB, INPUT_PULLUP);
  attachInterrupt (digitalPinToInterrupt (interruptPinA), ISR_EncoderA, CHANGE);
  attachInterrupt (digitalPinToInterrupt (interruptPinB), ISR_EncoderB,CHANGE);
  pinMode (DirPinl, OUTPUT);:
  pinMode (DirPin2, OUTPUT):
  // for second driver
  pinMode (interruptPinC, INPUT_PULLUP]
  pinMode (interruptPinD, INPUT_PULLUP]
  attachInterrupt (digitalPinToInterrupt (interruptPinC), ISR_EncoderC, CHANGE);
  attachInterrupt (digitalPinToInterrupt (interruptPinD), ISR_EncoderD, CHANGE);
  pinMode (DirPin3, OUTPUT);
  pinMode (DirPin4, OUTPUT);
  
  c1i();
  TCCRIA = 0;
  TCCRIB = 0;
  TCNT1 =0;
  OCRIA = 12499; //Prescaler = 64
  TCCRIB |= (1 << WGM12);
  TCCRIB |= (1 << CS11 | 1 << CS10):
  TIMSK1 |= (1 << OCIEIA);
  sei();

}

void leop() {
  if (count > count_prev) {
  t =millis();  // b7sb el wa2t mn bdayt el loop
  Theta = EncoderCountl / 1200.0;  // el one revolution feeha
  Theta2 = EncoderCount2 / 1200.0;
  a = (t - t_prev): // theta 3ady
  RPM_d = -150 ;  // desired RPM I want
  RPM_d2 = -150;
  
  
  RPM = (Theta - Theta_prev)/ (dt / 1000.0)*60 ;  // b2sm 31a alf 34an a7w1 milly to seconds and then €0 to get per menuite
  RPM2 = (Theta2 - Theta_prev2) / (dt / 1000.0) * 60;
  e= RMP_d - RPM;
  e2= RMP_d - RPM2;
  
  inte = inte_prev + (dt (e + e_prev) / 2) ;
  inte2 inte_prev2 + (dt * (e2 + e_prev2) / 2);
  
  V = kp * e + Ki*inte + (kd * (e — e_prev)/ dt);
  V = kp2 * e2 + Ki2 *inte2 + (kd2 * (e2 — e_prev2)/ dt);

  if (V > Vmax) {
    V = Vmax;
    inte = inte_prev;
    }
  if (V < Vmin) {
    V = Vmin;
    inte= inte_prev;
    }
  if (V2 > Vmax) {
    V2 = Vmax;
    inte2 = inte_prev2;
    }
  if (V2 < Vmin) {
    V2 = Vmin;
    inte2 = inte_prev2;
    }

  WriteDriverVoltage1(V,Vmax);
  WriteDriverVoltage2(V2,Vmax);
  Serial.print (RPM d); Serial.print ("\t") ;
  Serial.print(RPM); Serial.print ("\t");
  Serial.print(V); Serial.println ("\t");
  /*
  Serial.print (RPM d); Serial.print ("\t") ;
  Serial.print(RPM); Serial.print ("\t");
  Serial.print(V); Serial.println ("\t");
  */
  
  Theta_prev = Theta;
  count_prev = count;
  t_prev = t;
  inte_prev = inte;
  e_prev=e;
  
  Theta_prev2 = Theta2;
  inte_prev2 = inte2;
  e_prev2=e2;
  }
}
