#include <PID_v1.h>

#define numRows 4
#define numCols 4
#define sensorPoints numRows*numCols
#define TogglePin 2
#define StopPin 3

//------------------Variables for Reading Input-----------------------
int rows[] = {A0, A1, A2,A3};
int cols[] = {11,10,9,8};
int incomingValues[sensorPoints] = {};
int thresholdValues[sensorPoints] = {};
volatile int minVals[sensorPoints]={};
int T = 500;  //sample time in milliseconds (ms)
volatile int minIndex;
int nopress =0;
bool tog = 0;


//---------------------------PID Variables----------------------------
//Define Variables we'll be connecting to
double testthreshold, Input, Output, MaxVal;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=1, consKi=0.05, consKd=0.25;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &testthreshold, consKp, consKi, consKd, REVERSE);


//-----------------------Arduino Setup-------------------------------
void setup() {
  pinMode(TogglePin, OUTPUT);
  pinMode(StopPin,OUTPUT);

  digitalWrite(TogglePin, LOW);

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(500);
  myPID.SetOutputLimits(-1020, 1020);
  //myPID.SetTunings(consKp, consKi, consKd);

  // cli();
  //   //--------------------set timer0 interrupt at 2kHz-------------------
  // TCCR0A = 0;// set entire TCCR0A register to 0
  // TCCR0B = 0;// same for TCCR0B
  // TCNT0  = 0;//initialize counter value to 0
  // // set compare match register for 2khz increments
  // OCR0A = 124;// = (16*10^6) / (2000*64) - 1 (must be <256)
  // // turn on CTC mode
  // TCCR0A |= (1 << WGM01);
  // // Set CS01 and CS00 bits for 64 prescaler
  // TCCR0B |= (1 << CS01) | (1 << CS00);   
  // // enable timer compare interrupt
  // TIMSK0 |= (1 << OCIE0A);
  
  // sei();

  // set all rows and columns to INPUT (high impedance):
  for (int i = 0; i < numRows; i++) {
  pinMode(rows[i], INPUT_PULLUP); /// enable internal pull-up resistor
  ///HIGH when  open, and LOW when closed
  }
  for (int i = 0; i < numCols; i++) {
  pinMode(cols[i], INPUT);  
  }

  Serial.begin(9600);

}


// void loop() {
//   tog = !tog;
//   toggling(tog);
//   delay(1000);
// }

//--------------------------------Main Loop--------------------------------
void loop() {       //Main Loop
  for (int colCount = 0; colCount < numCols; colCount++) {
  pinMode(cols[colCount], OUTPUT); // set as OUTPUT
  digitalWrite(cols[colCount], LOW); // set LOW
  
  for (int rowCount = 0; rowCount < numRows; rowCount++) {
  incomingValues[colCount * numRows + rowCount] = analogRead(rows[rowCount]); // read INPUT
  }// end rowCount
  
  pinMode(cols[colCount], INPUT); // set back to INPUT!
 
  }// end colCount
  //Serial.print(sensorPoints);
  // Print the incoming values of the grid:
  // for (int i = 0; i < sensorPoints; i++) {
  // Serial.print(incomingValues[i]);
  
  // if(i<sensorPoints-1)
  // Serial.print("\t");
  // }
  // float P1 = (analogRead(A5)/1024.0 - 0.1)*100.0/0.8;
  // Serial.print("\t"); Serial.print(P1); Serial.print("\t");
  // Serial.println();
  // delay(750);

  int a=0;
  for(int i = 0; i < sensorPoints; i++)
  {
    //Serial.print(incomingValues[i]);
    if(incomingValues[i]<300) {//this threshold value of 250 can be changed to adjust the sensitivity
      Serial.print("grid no  ");
      Serial.print(i+1);
      Serial.print("  : ");
      Serial.println(incomingValues[i]);
      if(i!=6&&i!=7&&i!=10&&i!=11){
      a=1;
      }
    }
    else  {
      Serial.print("grid no  ");Serial.print(i+1);  Serial.print("  ");
      Serial.println(incomingValues[i]);
    }
  }
  if(a!=1)  {
    Serial.println("no press");
    nopress++;
    if(nopress%6==0)
    {
      Serial.println("Depressurizing");
      toggling(0);
      //delay(1000);
      toggling(1);
    }
    a=0;
  }
  ThresholdAdjustments();
  PressureAdjustments();
  if(a==1){
    TestingAlgo(minIndex);
    nopress =5;
  }
  delay(T);

}

void ThresholdAdjustments() {        // ReInitializing threshold Arrays
    for(int i =0;i<sensorPoints;i++) {   //Setting up the Max and Min Value arrays
      thresholdValues[i]=0;
      minVals[i]=1020;
  }
}


void PressureAdjustments() {      //Adjusting min and threshold value arrays
  // incomingValues[sensorPoints] = {};
  // thresholdValues[sensorPoints] = {};
  // minVals[sensorPoints]={};

  // 7 & 11 have same off trend, and 8 & 12
  for(int i = 0; i < sensorPoints; i++)
  {
    if(i==6||i==7||i==10||i==11){
      thresholdValues[i]=1020;
      minVals[i]=1020;
      continue;
    }
    if(thresholdValues[i] < incomingValues[i] )
      thresholdValues[i]=incomingValues[i];
    if(minVals[i]  > incomingValues[i] )
      minVals[i]=incomingValues[i];
  }
  int minVal=minVals[0];
  for (int i = 0; i < sensorPoints; i++) {
      if (minVals[i] < minVal) {
         minVal = minVals[i];
         minIndex = i;
      }
   }
}

void TestingAlgo(int index){     //Calls PID Controller and Toggling
  Input = minVals[index];

  Serial.print("Input : ");
  Serial.println(Input);
  MaxVal = thresholdValues[index];
  testthreshold = 250;
  double gap = abs(testthreshold-Input); //distance away from setpoint
  if(gap<30) {  
    //we're close to setpoint, use conservative tuning parameters
    myPID.SetTunings(consKp, consKi, consKd);
  }
  else {
     //we're far from setpoint, use aggressive tuning parameters
     myPID.SetTunings(aggKp, aggKi, aggKd);
  }
  myPID.Compute();
  Serial.print("Output : ");
  Serial.println(Output);
  if(Output>80) {
    toggling(1);     //keep pressurizing
  }
  if(Output<-80) {
    toggling(0);    //depressurizing
    Serial.println("Too much pressure, Forced Depressurization");
    //toggling(1);
  }
  if(Output>-80 && Output <80){
    stoptoggle();
    //toggling(0);
  }
}

void toggling(bool val)          //Triggers Pressurizing or DePressurizing ISR
{
  if(val == 1)
    digitalWrite(TogglePin, HIGH);
  if(val == 0)
  {
    digitalWrite(TogglePin, LOW);
    delay(1000);
  }
}
void stoptoggle() {             //Stop changing pressure
  digitalWrite(StopPin, HIGH);
  Serial.println("Holding Pressure");
  delay(3000);        //hold for 3 seconds
  digitalWrite(StopPin, LOW);
}

// //---------------------------ISRS---------------------
// void ISR(TIMER0_COMPA_vect){//timer0 interrupt 2kHz toggles pin 8
// //generates pulse wave of frequency 2kHz/2 = 1kHz (takes two cycles for full wave- toggle high then toggle low)
//   count++;
// }