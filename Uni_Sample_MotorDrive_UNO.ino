//This code has been modified by Habib to Include the Acc. Pedal
// The input range of the Acc_Pedal is from 180-871, the selected range in the code is 185-865
// Digital Gate Enable Signal pins 
byte DE_A_p=2; byte DE_B_p=4; byte DE_C_p=7;   
// PWM Signal pins
byte PWM_A_p=9; byte PWM_B_p=10; byte PWM_C_p=11;
  
// Hall Effect Sensors: pins, sensor values(int), and status (bool) 
byte H_A_p=A0; byte H_B_p=A1; byte H_C_p=A2; // pins
//byte H_A_p = 3; byte H_B_p = 5; byte H_C_p = 6; // Digital Pin Assignment for Hall Effect Sensors
int H_A_v=0; int H_B_v=0; int H_C_v=0;
int Hthresh=512; //sensor threshold, range=0 to 1023.
//int Hthresh=5;// Digital Threshold
bool H_A=false; bool H_B=false; bool H_C=false;
  
// Accelerator signal pin and value
byte Acc_p=A3; int Acc; float Vmax=5.0;
float D=0.0; float Dref=0.0; // Duty Ratio:
int Acc_tc; int Acc_tp; int Acc_tpp;
// Main Switch Pin and state:
byte MainSW_p=12; bool MainSW=false;
// FwdRev Switch Pin and state:
byte FwdRevSW_p=8; bool FwdRevSW=true; // true is fwd, false is rev

// Current and Previous States:
byte CS=0; byte PS=0;

// Delay Measurement Vars
int td1 = 0;
int td2 = 0;

void setup() {
  //Serial.begin(9600);
  pinMode(Acc_p, INPUT);
  pinMode(MainSW_p, INPUT);
  pinMode(FwdRevSW_p, INPUT);
  pinMode(DE_A_p, OUTPUT); pinMode(DE_B_p, OUTPUT); pinMode(DE_C_p, OUTPUT);
  pinMode(PWM_A_p, OUTPUT); pinMode(PWM_B_p, OUTPUT); pinMode(PWM_C_p, OUTPUT);
  pinMode(H_A_p, INPUT); pinMode(H_B_p, INPUT); pinMode(H_C_p, INPUT);
//---------------------------------------------- Set PWM frequency for D9 & D10 ------------------------------
TCCR2B = TCCR2B & B11111000 | B00000001;    // set timer 2 divisor to     1 for PWM frequency of 31372.55 Hz

//---------------------------------------------- Set PWM frequency for D11 & D12 -----------------------------
TCCR1B = TCCR1B & B11111000 | B00000001;    // set timer 1 divisor to     1 for PWM frequency of 31372.55 Hz
  
//---------------------------------------------- Change The Prescale of AnalogRead ------------------------------
  // defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// set prescale to 16
sbi(ADCSRA,ADPS2) ;
cbi(ADCSRA,ADPS1) ;
cbi(ADCSRA,ADPS0) ;

}

void loop() {
  //Serial.println(Acc_tc);
 MainSW=digitalRead(MainSW_p);
 FwdRevSW=digitalRead(FwdRevSW_p); //use a pull up res. if no sw connected (fwd=1, rev=0)
 FwdRevSW=true;
 if (MainSW){
  //td1 = micros();
  Acc_tc=analogRead(Acc_p); // ET = 116uS, After Changing the prescale ET = 20uS
  Acc=(Acc_tc+Acc_tp+Acc_tpp)/3; // ET = 20uS
  Acc_tpp=Acc_tp; Acc_tp=Acc_tc; // ET = 4uS
  //Dref=map(Acc,0,1023,0,255); // ET = 56uS
  //Dref = Acc*0.2492668622; //ET = 16uS
  Dref = (Acc*(0.375)- 69.375); // ET = 26uS
  if(Dref<0){
    Dref = 0;
    }else{
      if(Dref > 255){
        Dref = 255;
          }
        }
  readHall();// Read hall effect sensors: ET = 20 uS
  CS=getState(); // determine the current state ET = 8uS
  D=Dref;
  setSwitches(); // ET = 4uS
  //td2 = micros();
  //Serial.println(td2-td1);
 } 
 else{turnOff(); CS=0;}
 PS=CS;
}

void readHall() {
  //H_A_v=analogRead(H_A_p); H_B_v=analogRead(H_B_p); H_C_v=analogRead(H_C_p);
  H_A_v=digitalRead(H_A_p); H_B_v=digitalRead(H_B_p); H_C_v=digitalRead(H_C_p);
  //H_A_v = digitalRead(H_A_p); H_B_v = digitalRead(H_B_p); H_C_v = digitalRead(H_C_p);
  if (H_A_v == HIGH) {H_A=true;}
  else {H_A=false;}
  if (H_B_v == HIGH) {H_B=true;}
  else {H_B=false;}
  if (H_C_v == HIGH) {H_C=true;}
  else {H_C=false;}
}    

byte getState() {
  byte State=0; // an incorrect state resulting from an incorrect Hall sensor reading
  if(FwdRevSW){// Clockwise Rotation
    if (!H_A && !H_B &&  H_C) {State=1;}
    if (!H_A &&  H_B &&  H_C) {State=2;}
    if (!H_A &&  H_B && !H_C) {State=3;}
    if ( H_A &&  H_B && !H_C) {State=4;}  
    if ( H_A && !H_B && !H_C) {State=5;}
    if ( H_A && !H_B &&  H_C) {State=6;}
  }
  else {// Counter-Clockwise Rotation
    if ( H_A &&  H_B && !H_C) {State=1;}
    if ( H_A && !H_B && !H_C) {State=2;}
    if ( H_A && !H_B &&  H_C) {State=3;}
    if (!H_A && !H_B &&  H_C) {State=4;}  
    if (!H_A &&  H_B &&  H_C) {State=5;}
    if (!H_A &&  H_B && !H_C) {State=6;}
  }
  return State;
}

void setSwitches() {
  // Turn off unused phase, e.g., C: Set D=0 for PWM_C
  // To turn on an upper switch,e.g. A: Set DE_A=1 and Set D=Dref for PWM_A 
  // To turn on a lower switch, e.g. B: Set DE_B=0 and Set D=255 for PWM_B (on all the time)
  
  switch (CS){ 
    case 1: // BU, CL
    analogWrite(PWM_A_p,0); digitalWrite(DE_A_p,false);// turn off unused phase
    digitalWrite(DE_B_p,true);  analogWrite(PWM_B_p,D); // turn on upper switch and set D=Dref for its PWM
    digitalWrite(DE_C_p,false); analogWrite(PWM_C_p,255);// turn on lower switch DE=0 and set D=255 for its PWM (on all the time)
    break;
    case 2: // AU, CL
    analogWrite(PWM_B_p,0);digitalWrite(DE_B_p,false);
    digitalWrite(DE_A_p,true);  analogWrite(PWM_A_p,D); 
    digitalWrite(DE_C_p,false); analogWrite(PWM_C_p,255);
    break;
    case 3: // AU, BL
    analogWrite(PWM_C_p,0); digitalWrite(DE_C_p,false);// turn off unused phase
    digitalWrite(DE_A_p,true);  analogWrite(PWM_A_p,D);
    digitalWrite(DE_B_p,false); analogWrite(PWM_B_p,255);
    break;
    case 4: // CU, BL
    analogWrite(PWM_A_p,0); digitalWrite(DE_A_p,false);// turn off unused phase
    digitalWrite(DE_C_p,true);  analogWrite(PWM_C_p,D); 
    digitalWrite(DE_B_p,false); analogWrite(PWM_B_p,255);
    break;
    case 5: // CU, AL
    analogWrite(PWM_B_p,0); digitalWrite(DE_B_p,false);// turn off unused phase
    digitalWrite(DE_C_p,true);  analogWrite(PWM_C_p,D); 
    digitalWrite(DE_A_p,false); analogWrite(PWM_A_p,255); 
    break;
    case 6: // BU, AL
    analogWrite(PWM_C_p,0); digitalWrite(DE_C_p,false);// turn off unused phase
    digitalWrite(DE_B_p,true);  analogWrite(PWM_B_p,D); 
    digitalWrite(DE_A_p,false); analogWrite(PWM_A_p,255);
    break;
    }
} 

void turnOff() {
  analogWrite(PWM_A_p,0);
  analogWrite(PWM_B_p,0);
  analogWrite(PWM_C_p,0);
}
