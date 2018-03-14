#include <Servo.h>

// LEDS attatched to 3 PWMS
#define LED_RED 9
#define LED_GREEN 10
#define LED_BLUE 11

// Button + FSM
int button = 2; // used for interrupt purposes
long debouncing_time = 150000; //Debouncing Time in useconds
volatile unsigned long last_valid; // In useconds
volatile bool state = 1; // 1: ON 0: LOW POWER

// motor instances and variables
Servo servo;
int flexSensor;
float servoPos;

// vars for filters
float iir_Av = 0;
int16_t cbuf[32];
uint8_t offset = 0;

// 12 poles for FIR filter
static const uint16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096};

void setup() 
{
  Serial.begin(9600);
  servo.attach(6); // PWM 6
  servo.write(0); // reset servo to original position
  pinMode(LED_RED, OUTPUT); pinMode(LED_GREEN, OUTPUT); pinMode(LED_BLUE, OUTPUT); 
  attachInterrupt(digitalPinToInterrupt(button), ISR_debounce, FALLING);
}

void loop() 
{
  // read sensor value (voltage divider - 10K)
  flexSensor = analogRead(0);

  // calculate filter values
  float iirStatus = iirFilter(flexSensor);
  int firVal = lowPassFIRFilter((int)flexSensor);

  // DO SOMETHING WITH PROCESSED VALUES
}

void driveMotor(int positionVal)
{
  servoPos = map(positionVal, 700, 900, 0, 180);
  //servoPos = constrain(flexSensor, 0, 180);

  // move servo based on position of flex sensor  
  if(servoPos < 180 && servoPos > 0)
    servo.write(servoPos);
  delay(20);
}
// test function which rotates servo motor 180 degrees when button is pressed
void testButton()
{
  pinMode(8, INPUT); // used to measure state of button
  
  // loop in test mode
  while(1)
  {
    // read button state
    int buttonState = digitalRead(8);
    
    // if button is pressed (assuming pulldown)
    if(buttonState)
      servo.write(180);
    
    // if button is not pressed (assuming pullup)
    else
      servo.write(0);
  }
}

// Low Pass IIR Filter
float iirFilter(int flexVal)
{
  //iir_Av += (((float)flexVal << 15) - iir_Av) >> 4);
  iir_Av += ((float)flexVal - iir_Av)/16;
  return iir_Av;
}

//  Low Pass FIR Filter (12 pole)
int lowPassFIRFilter(int newSample)
{  
  // store new value in array
  cbuf[offset] = newSample;

  // start with middle point
  int32_t z = mul16(FIRCoeffs[11], cbuf[(offset - 11) & 0x1F]);

  // loop through -> mulltiply stored values by pole coefficants
  for (uint8_t i = 0 ; i < 11 ; i++)
  {
    z += mul16(FIRCoeffs[i], cbuf[(offset - i) & 0x1F] + cbuf[(offset - 22 + i) & 0x1F]);
  }

  // increase and wrap offset condition
  offset++;
  offset %= 32;

  // scale FIR output
  return(z >> 15);
}

//  Integer multiplier
int32_t mul16(int16_t x, int16_t y)
{
  return((long)x * (long)y);
}

void turnOnLED(int ledColour)
{
  PORTB = (1 << (ledColour - 8));
}

// Debouncing function - determines whether button press is valid
void ISR_debounce()
{
  if((long)(micros() - last_valid) >= debouncing_time) 
  {
    changeState();
    last_valid = micros();
  }
}

// change state from ON <-> LOW POWER
void changeState()
{
  state = !state;
  digitalWrite(9, state);
}
