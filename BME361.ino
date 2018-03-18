#include <Servo.h>
Servo servo;           // servo instance

// Pin definitions
#define read_flex 0                             // ANALOG - flex sensor reading
#define batt_read 1                             // ANALOG - battery monitoring
#define button 2                                // DIGITAL - interrupt purposes
#define lin_servo 6                             // DIGITAL - control servo over PWM
#define LED_RED 9                               // DIGITAL - turned on in low power
#define LED_GREEN 10                            // DIGITAL - turned on in active state
#define LED_BLUE 11                             // DIGITAL - turned on standby mode

// Battery and Batt state
bool batt_state;                                // 1 = HIGH | 0 = LOW POWER
const float referenceVolts = 5.0;               // the default reference on a 5-volt board
float adc_res = 1023.0;                         // resolution of analog -> digital converter (2^10)

// Button + FSM
long debouncing_time = 150000;                  // Debouncing Time in useconds
volatile unsigned long last_valid;              // In useconds
volatile bool state = 0;                        // 1: ON 0: LOW POWER
volatile bool int_flag = 0;                     // used to signal whem button was just pressed

// Flex sensor and filters
int flexSensor;                                 // analog read of flexsensor
float iir_Av = 0;                               // used to track IIR value
int16_t cbuf[32];                               // buffer storing latest 32 values
uint8_t offset = 0;
// 12 poles for FIR filter
static const uint16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096};

void setup() 
{
  Serial.begin(9600);
  servo.attach(lin_servo); // PWM 6
  servo.write(0); // reset servo to original position
  pinMode(LED_RED, OUTPUT); pinMode(LED_GREEN, OUTPUT); pinMode(LED_BLUE, OUTPUT); 
  attachInterrupt(digitalPinToInterrupt(button), ISR_debounce, FALLING);
}

void loop() 
{
  // read sensor value (voltage divider - 10K)
  flexSensor = analogRead(read_flex);

  // calculate filter values
  float iirStatus = iirFilter(flexSensor);
  batt_state = checkBattery();
  changeState();

  // check if interrupt or low battery state or detected grip/release
  if(int_flag || !batt_state)
  {
    if(!batt_state || !state)
      driveMotor(0);
    else
      driveMotor(180);
    int_flag = 0;
  }  
}

void driveMotor(int servoPos)
{
  // move servo bus restrict to (0,180)
  if(servoPos <= 180 && servoPos >= 0)
    servo.write(servoPos);
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

void turnOnLED(int ledColour)
{
  PORTB = (1 << (ledColour - 8));
}

// change state from ON <-> LOW POWER
void changeState()
{
  if(!batt_state)
  {
    turnOnLED(LED_RED);
    state = 0; // reset state to low power -- require user to press button after plugging in battery
  }
  else if(state)
    turnOnLED(LED_GREEN);
  else if(!state)
    turnOnLED(LED_BLUE);
}

bool checkBattery()
{
// read value from sensor and divide it by the resolution, multiply by reference voltage
  float volts = (analogRead(batt_read) / adc_res)*referenceVolts*2; 
  if(volts >= referenceVolts)
    return true;
  else
    return false;
}

// Debouncing function - determines whether button press is valid
void ISR_debounce()
{
  if((long)(micros() - last_valid) >= debouncing_time) 
  {
    state = !state; // change from low power <-> active
    int_flag = 1;
    last_valid = micros();
  }
}

//  Integer multiplier
int32_t mul16(int16_t x, int16_t y)
{
  return((long)x * (long)y);
}
