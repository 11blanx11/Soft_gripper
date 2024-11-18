//#include <Keypad.h>
//const byte ROWS = 4; //four rows
//const byte COLS = 4; //four columns
//char keys[ROWS][COLS] = {
//  {'1','2','3','A'},                                                                                                                                                                                
//  {'4','5','6','B'},
//  {'7','8','9','C'},
//  {'O','F','E','D'}
//};
//
//byte rowPins[ROWS] = {11,10,9, 8}; //connect to the row pinouts of the keypad
//byte colPins[COLS] = {5, 4, 3, 2}; //connect to the column pinouts of the keypad
////Create an object of keypad
//Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

#include <Wire.h>
//--------------PINS--------------
const int forwards_pin = 11;        //changed from pin 12
const int backwards_pin = 12;
const int interruptPin = 2;
const int stopPin = 3;
const int WritePin = 7;
//------------VARIABLES-------------
const int pwm= 9;
//int value=0;
int count =0;
//bool Mode = 0; 
bool execute;
bool test;
bool toggled;
bool ReadVal;
bool PressurizeStart;

int val;

void setup() {
  analogReference(DEFAULT);


  cli();//stop interrupts

  pinMode(LED_BUILTIN, OUTPUT);

  //------------------Timer 1 for PWM------------------------
  TCCR1A = _BV(COM1A1) | _BV(COM1B1);       // Using Timer 1 , Clear OC1 while upcounting, Set OC1 while down counting
  TCCR1B = _BV(WGM13) | _BV(CS10);          
  ICR1 = 400 ; // 10 bit resolution         //Max Value
  OCR1A = 0; // vary this value between 0 and 400 for 10-bit precision
  OCR1B = 328; // vary this value between 0 and 400 for 10-bit precision 
  pinMode (9, OUTPUT);
  pinMode(forwards_pin, OUTPUT);//set relay as an output
  pinMode(backwards_pin, OUTPUT);

  sei();

  pinMode(interruptPin, INPUT_PULLUP);
  pinMode(stopPin, INPUT_PULLUP);
  pinMode(WritePin, OUTPUT);
  digitalWrite(WritePin, LOW);
  attachInterrupt(digitalPinToInterrupt(interruptPin), ISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(stopPin), ISR2, CHANGE);

  Pressurize(180);
  
  Serial.begin(9600);
  Wire.begin(9);
  Wire.onReceive(receiveEvent);

  

}

void loop() {

  if(execute)
  {
    Serial.println("Stable Pressure Detected");
    delay(3000);
    DePressurize();
    execute=false;
  }

  if(test)
  {
    //digitalWrite(LED_BUILTIN,Mode);
    Serial.println("ISR Triggered ");
    DePressurize();
    test=false;
  }

  if(ReadVal)           //Printing the read value
  { 
    Serial.print("PWM Value : ");
    Serial.println(val);
    Pressurize(val);
    ReadVal=false;
  }

  if(PressurizeStart)
  {
    Serial.print("Pressurization Started with : ");
    Serial.println(val);
    PressurizeStart = false;
  }

  if(toggled)
  {
    Serial.println("Interrupt pin Toggled");
    toggled = false;
  }

}

//---------------------------Functions--------------------------

void Pressurize(int PWM_Value) {     //Pressurize according to Output
  PWM_Value = map(PWM_Value,0,255,0,300);
  val=PWM_Value;
  OCR1A/*pin 9*/ =(300-PWM_Value)/* speed 0 to 350 */;      //Increase pressure

  digitalWrite(forwards_pin, HIGH);
  digitalWrite(backwards_pin, LOW);
  PressurizeStart=true;
  delay(150); // wait 2 seconds
  OCR1A/*pin 9*/ = 400/* speed 0 to 400 */;
  digitalWrite(forwards_pin, LOW);
  digitalWrite(backwards_pin, LOW);

  delay(250);
  digitalWrite(WritePin,HIGH);
  toggled = true;
  delay(1);
  digitalWrite(WritePin,LOW);
}

// void Pressurize() {                   // Standard Pressurize
//   val = 247;
//   OCR1A/*pin 9*/ =(350-val)/* speed 0 to 350 */;      //Increase pressure
//   digitalWrite(forwards_pin, HIGH);
//   digitalWrite(backwards_pin, LOW);
//   PressurizeStart = true;
//   delay(100); // wait 2 seconds
//   OCR1A/*pin 9*/ = 400/* speed 0 to 400 */;
//   digitalWrite(forwards_pin, LOW);
//   digitalWrite(backwards_pin, LOW);

//   delay(250);
//   digitalWrite(LED_BUILTIN,HIGH);
//   toggled = true;
//   delay(50);
//   digitalWrite(LED_BUILTIN,LOW);
//   digitalWrite(backwards_pin, HIGH);
// }

void DePressurize() {
  OCR1A/*pin 9*/ = 400/* speed 0 to 400 */;
  Serial.println("Depressurizing");
  digitalWrite(forwards_pin, LOW);
  digitalWrite(backwards_pin, HIGH);
  delay(2000); // wait 2 seconds
  Pressurize(180);
}


//------------------------------ISRs-------------------------

void ISR1() {
  //OCRtoggle();
  test=true;
  //test=true;
}

void ISR2(){
  digitalWrite(forwards_pin, LOW);  //Hold Pressure
  digitalWrite(backwards_pin, LOW);
  execute= true;
}

void receiveEvent(int bytes) 
{
  val = Wire.read();
  //Pressurize(val);
  ReadVal = true;
}


 