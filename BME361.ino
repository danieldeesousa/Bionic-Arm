#include <Servo.h>

// motor instances and variables
Servo servo;
int flexSensor;
float servoPos;


float iir_Av = 0;
int16_t cbuf[32];
uint8_t offset = 0;

// 12 poles
static const uint16_t FIRCoeffs[12] = {172, 321, 579, 927, 1360, 1858, 2390, 2916, 3391, 3768, 4012, 4096};

void setup() 
{
  Serial.begin(9600);
  servo.attach(9);
  servo.write(0); // reset servo to original position
}

void loop() 
{
  // read sensor value (voltage divider - 10K)
  flexSensor = analogRead(0);

  // calculate filter values
  float iirStatus = iirFilter(flexSensor);
  int firVal = lowPassFIRFilter((int)flexSensor);
  
  Serial.println(firVal);

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
