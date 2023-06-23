

//defines the BAUDRATE
#define BAUDRATE 9600


//defines the various pins for this setup
//defines the pin for the Encoder pin 1
#define ENCODER_PIN_1 2
//defines the pin for the Encoder pin 2
#define ENCODER_PIN_2 3
//defines the pin for the MOTOR, for pulse width modulation
#define MOTOR_PWM_PIN 4

//defines the specifications for the interrupts
#define INTERRUPT_0 0
#define INTERRUPT_1 1



//creates the variables to store the current time in microseconds
volatile uint64_t currentTime = 0;
volatile uint64_t previousTimeSensor = 0;
volatile uint64_t previousTimeSerial = 0;




//creates the number of microseconds needed between samples 
const uint32_t sensorSampleTime = 20000;
//defines the sample rate, which with a sampling time of 1000 Microseconds
//is 1 Kilohertz
const uint32_t sensorSampleRate = 50;
//sets the sample time for the serial terminal in microseconds. 0.1 seconds.
const uint32_t serialSampleTime = 200000;
//sets the serial sample rate. 100 Hertz
const uint32_t serialSampleRate = 5;


//creates the variable to store the current encoder position in ticks
volatile long currentPosition = 0;
//creates the variable to store the previous encoder position in ticks
volatile int32_t previousPosition = 0;

//conversion factor from ticks to radians
const double ticksToRadians = 190.0;

//our conversion factor from ticks to Radians
const double ticksToMilliradians = 5.26316;

//our angular velocity in ticks per second
volatile int32_t angularVelocityTicks = 0;
//creates angular velocity in ticks but as a double
volatile double angularVelocityTicksDouble = 0.0;
//our angular velocity in milliradians per second
volatile double angularVelocityRadians = 0.0;


void setup() {
  Serial.begin(BAUDRATE);

  //sets the pin modes for the encoder pins
  pinMode(ENCODER_PIN_1, INPUT_PULLUP);
  pinMode(ENCODER_PIN_2, INPUT_PULLUP);
  //sets the motor pin as an output
  pinMode(MOTOR_PWM_PIN, OUTPUT);


  //attaches both interrupts
  attachInterrupt(INTERRUPT_0, ai0, RISING);
  attachInterrupt(INTERRUPT_1,  ai1, RISING);


}

void loop() 
{
  /////////////////////////////////////////////////////////////////////////////////
  //gets the current values for both microseconds and milliseconds
  currentTime = micros();
  /////////////////////////////////////////////////////////////////////////////////



  //Turns the motor on
  analogWrite(MOTOR_PWM_PIN, 256);

  //checks if time has elapsed to take a sensor sample
  if((currentTime - previousTimeSensor) >= sensorSampleTime)
  {
    //Serial.print("Current Position: ");
    //Serial.println(currentPosition);
    //calculates the angular velocity
    angularVelocityTicks = (currentPosition - previousPosition)*sensorSampleRate;
    //gets angular Velocity Ticks as a double
    angularVelocityTicksDouble = (double)angularVelocityTicks;
    //calculates angular velocity in radians per second from angular velocity in ticks per second
    angularVelocityRadians = angularVelocityTicksDouble/ticksToRadians;
        
    //Serial.println(angularVelocity);
    //updates the previous position to the current position
    previousPosition = currentPosition;

    //makes sure to reset previousTimeSensor to the current time
    previousTimeSensor = currentTime;
  }

  //Serial.println((currentTime - previousTimeSerial));
  //Serial.println(serialSampleTime);
  //checks if we've passed the time for sending the serial message
  if((currentTime - previousTimeSerial) >= serialSampleTime)
  {
    //prints the angular Velocity
    //Serial.print("Current Position: ");
   // Serial.println(currentPosition);
    Serial.print("Angular Velocity (Ticks per second): ");
    Serial.println(angularVelocityTicks);
    Serial.print("Angular Velocity Double: ");
    Serial.println(angularVelocityTicksDouble);
    Serial.print("Angular Velocity (Radians per second): ");
    Serial.println(angularVelocityRadians);
    //Serial.println("Previous Position: ");
    //Serial.println(previousPositionRadians);
    //makes sure to reset the previous time serial
    previousTimeSerial = currentTime;
  }


}



  void ai0() {
  // ai0 is activated if DigitalPin nr 2 is going from LOW to HIGH
  // Check pin 3 to determine the direction
  if(digitalRead(3)==LOW) {
  currentPosition++;
  //Serial.println ("3 added");
  }else{
  currentPosition--;
  //Serial.println ("3 subbed");
  }
  }
   
  void ai1() {
  // ai0 is activated if DigitalPin nr 3 is going from LOW to HIGH
  // Check with pin 2 to determine the direction
  if(digitalRead(2)==LOW) {
  currentPosition--;
  //Serial.println ("2 added");
  }else{
  currentPosition++;
  //Serial.println ("2 subbed");
  }
  }



//What are the similarities between a frog and a bicycle?
//They both have handle bars, except for the frog.