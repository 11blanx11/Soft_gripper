#include <PID_v1.h>
#include<Wire.h>

#define numRows 4
#define numCols 4
#define sensorPoints numRows*numCols
#define TogglePin 2
#define StopPin 3
#define ReadPin 7
#define PWM_Control 6

//------------------Variables for Reading Input-----------------------
int rows[] = {A0, A1, A2,A3};
int cols[] = {11,10,9,8};
int incomingValues[sensorPoints] = {};
int thresholdValues[sensorPoints] = {};
volatile int minVals[sensorPoints]={};
int T = 150;  //sample time in milliseconds (ms)
volatile int minIndex;
volatile int RxByte;
int nopress =0;
int count=0;
int button;
double AggVol=0;
double deltaVol;
bool readval = false;
bool requestflag = false;


unsigned long startmillis;  
unsigned long currentmillis;
unsigned long time;
unsigned long period;


//---------------------------PID Variables----------------------------
//Define Variables we'll be connecting to
double SetPoint, Input, Output, MaxVal;
double InputResistance, CorrespondingVolume;

double ResistanceSetpoint = 300;
double dmgthreshold = 180;

//Define the aggressive and conservative Tuning Parameters
double aggKp=4, aggKi=0.2, aggKd=1;
double consKp=25, consKi=0.0, consKd=0.0;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &SetPoint, consKp, consKi, consKd, DIRECT);


//-----------------------Arduino Setup-------------------------------
void setup() {
  analogReference(DEFAULT);
  cli();

  pinMode(TogglePin, OUTPUT);
  // pinMode(StopPin,OUTPUT);

  pinMode(ReadPin, INPUT_PULLUP);

  digitalWrite(TogglePin, HIGH);

  //attachInterrupt(digitalPinToInterrupt(ReadPin), ISR1, CHANGE);


  //----------PIDSetup--------------------
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(T);
  myPID.SetOutputLimits(0,180);
  time = millis();
  myPID.SetTunings(consKp, consKi, consKd);



  // set all rows and columns to INPUT (high impedance):
  for (int i = 0; i < numRows; i++) {
  pinMode(rows[i], INPUT_PULLUP); /// enable internal pull-up resistor
  ///HIGH when  open, and LOW when closed
  }
  for (int i = 0; i < numCols; i++) {
  pinMode(cols[i], INPUT);  
  }

  Serial.begin(9600);
  Wire.begin();

  sei();
  //ReadSens();
}


//--------------------------------Main Loop--------------------------------
void loop() {       //Main Loop

  // if(readval)
  // {
  //   Serial.print("Recieved Event from Motor at : ");
  //   Serial.println(millis());
  //   readval = false;
  //   ReadSens();    
  // }

  button = digitalRead(ReadPin);
  if(button)
  {
    Serial.print("Recieved Event from Motor at : ");
    Serial.print(button);
    Serial.println(millis());
    ReadSens();
    delay(100);
    cli();
  }


}

void ThresholdAdjustments() {        // ReInitializing threshold Arrays
    for(int i =0;i<sensorPoints;i++) {   //Setting up the Max and Min Value arrays
      thresholdValues[i]=0;
      minVals[i]=1020;
  }
}


void PressureAdjustments() {      //Adjusting min and threshold value arrays

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




//----------------------------------- FUNCTIONS -------------------------------------

void ReadSens()
{

  for (int colCount = 0; colCount < numCols; colCount++) {
    pinMode(cols[colCount], OUTPUT); // set as OUTPUT
    digitalWrite(cols[colCount], LOW); // set LOW

    for (int rowCount = 0; rowCount < numRows; rowCount++) {
      incomingValues[colCount * numRows + rowCount] = analogRead(rows[rowCount]); // read INPUT
    }// end rowCount

    pinMode(cols[colCount], INPUT); // set back to INPUT!

  }// end colCount

  ThresholdAdjustments();
  PressureAdjustments();
  sei();
  TestingAlgo(4);

}

void TestingAlgo(int index){     // Prints Input Values and then Calls PID Controller and sends PWM Value to motor and toggling
  InputResistance = incomingValues[index];

  CorrespondingVolume = pow((2401/InputResistance),(1/1.07));

  Input = CorrespondingVolume;

  Serial.print("Input Resistance : ");
  Serial.print(InputResistance);
  Serial.print(" , Input : ");
  Serial.print(Input);
  Serial.print(" , Estimated Volume : ");
  Serial.print(AggVol);
  Serial.print(" , Input Time ");
  Serial.print(millis());

  MaxVal = thresholdValues[index];

  SetPoint = pow((2401/ResistanceSetpoint),(1/1.07));
  double gap = abs(SetPoint-Input); //distance away from setpoint
  myPID.SetTunings(consKp, consKi, consKd);

  myPID.Compute();
  Serial.print(" ,Output : ");
  currentmillis=millis();
  Serial.print(Output);
  Serial.print(" ,Output Time : ");
  Serial.println(currentmillis);

  deltaVol = (0.022875)*(Output/255)*T;
  AggVol = AggVol + deltaVol;


  Wire.beginTransmission(9);
  Wire.write(int(Output));              
  Wire.endTransmission();

  if(int(Output)!=0)
  {
    count =0;
  }

  if(int(Output)==0)
  {
      count++;
      if(count%10==0)
      {
        AggVol = 0;
        toggling(false);
      }
  }
  if (AggVol > 10)
  {
        AggVol = 0;
        toggling(false);
  }
  if (InputResistance < 240)
  {
        AggVol = 0;
        toggling(false);
  }
}


void toggling(bool val)          //Triggers Pressure or Depressure
{
  //if(val == 1)
  
  if(!val)
  {
    Serial.println("Depressurizing");
    digitalWrite(TogglePin, LOW);
    delay(1000);
    digitalWrite(TogglePin, HIGH);
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

void ISR1() {
  if(digitalRead(ReadPin))
  {
    readval=true;
  }
  
}

