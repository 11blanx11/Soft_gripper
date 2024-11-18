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


const int forwards_pin = 11;        //changed from pin 12
const int backwards_pin = 12;
const int interruptPin = 2;
const int stopPin = 3;
const int pwm= 9;
int value=0;
int count =0;
bool Mode = 0; 
bool execute;
bool test;

void setup() {
 
  cli();//stop interrupts
  Serial.begin(9600);

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
  attachInterrupt(digitalPinToInterrupt(interruptPin), ISR1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(stopPin), ISR2, CHANGE);

  Pressurize();
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
    Mode =!Mode;
    digitalWrite(LED_BUILTIN,Mode);
    Serial.println("ISR Triggered ");
    test=false;
  }


  // Pressurize();
  // delay(2000); // wait 2 seconds

  // DePressurize();
  // delay(2000); // wait 2 seconds
}

//---------------------------Functions--------------------------

void Pressurize() {
  OCR1A/*pin 9*/ =200/* speed 0 to 400 */;      //Increase pressure
  digitalWrite(forwards_pin, HIGH);
  digitalWrite(backwards_pin, LOW);
}
void DePressurize() {
  OCR1A/*pin 9*/ = 0/* speed 0 to 400 */;       //Remove pressure
  digitalWrite(forwards_pin, LOW);
  digitalWrite(backwards_pin, HIGH);
  //delay(1000);
}


void OCRtoggle()  {                             // Call Pressurize or Depressurize
  digitalWrite(LED_BUILTIN, HIGH);
  if(digitalRead(interruptPin)) {
    Pressurize();
  }
  else if(!digitalRead(interruptPin)) {
    DePressurize();
  }
  // if(OCR1A == 200)
  // {
  //   DePressurize();
  //   Serial.println("Cutting Pressure")
  // }
  //   if(OCR1A == 0)
  // {
  //   Pressurize();
  //   Serial.println("Adding Pressure")
  // }
}

//------------------------------ISRs-------------------------

void ISR1() {
  OCRtoggle();
  test=true;
}

void ISR2(){
  digitalWrite(forwards_pin, LOW);  //Hold Pressure
  digitalWrite(backwards_pin, LOW);
  execute= true;

}
 