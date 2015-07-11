
// Quentin McRae
// Nov 15, 2013

// initialize barometer stuff ...
#include <SFE_BMP180.h>
#include <Wire.h>
// You will need to create an SFE_BMP180 object, here called "pressure":
SFE_BMP180 pressure;
double baseline;   // baseline pressure to work from ...

/* Hardware connections for barometer:
 - (GND) to GND
 + (VDD) to 3.3V
 (WARNING: do not connect + to 5V or the sensor will be damaged!)
 
 For Uno, Nano, Pro ...
  SDA to A4
  SCL to A5    */



// Initialize other stuff
int left_motor[]  = {3, 2,  4 }; // 3-pins controlling motor thru h-bridge
int right_motor[] = {5, A2, A3}; // 3-pins controlling motor thru h-bridge
int down_motor[]  = {6, 8, 9}; // 3-pins controlling motor thru h-bridge
int left_motor_signal  = 0;      // Initial motor signal (initiallly stopped until there is a reason to move)
int right_motor_signal = 0;      // Initial motor signal (initiallly stopped until there is a reason to move)
int down_motor_signal  = 0;      // Initial motor signal (initiallly stopped until there is a reason to move)
int aux_command;                 // < -50 - switch to baro, > +50, which to RC
int throttle_command;            // -100 is full-speed backward, +100 is full-speed forward
int steering_command;            // +100 is full right turn, -100 is full left turn, 0 is straight
int rc_down_command;             // +100 is full speed, 0 is off
int down_command;                // +100 is full speed, 0 is off
int aux_pin = 10;                 // Pin that the aux input comes in on (used to determine RC vs. baro altitude control)
int throttle_pin = 7;            // Pin that the throttle input comes in on
int steering_pin = 11;           // Pin that the steering input comes in on
int down_pin = 12;               // Pin that the altitude-fan input comes in on
int battery_pin = A1;            // Pin to read the battery voltage
int led_pin = 13;

// Initialize min and max radio inputs ...
int min_pulse_throttle = 1200;  // These values are to set the full left/right and forward/reverse pulse values from the radio
int max_pulse_throttle = 1940;  // These should probably be found by some sort of startup calibration
int min_pulse_steering = 1100;
int max_pulse_steering = 1940;
int min_pulse_down = 1180;
int max_pulse_down = 1946;
int min_pulse_aux = 1100;
int max_pulse_aux = 1780;

// Define the min/max pwm signal sent - this is so we can control the overall speed, as well as easily be able to deal with
//  larger voltage batteries without frying the motors 
int min_pwm_signal_down = -255;
int max_pwm_signal_down =  255;
int min_pwm_signal_throttle = -150;
int max_pwm_signal_throttle =  150;

//  Define a "dead-band", or a range where we don't try to react to radio inputs, so the dumb thing will turn off when we want it to!
int dead_band_throttle = 10;    //  ± x% deadband
int dead_band_steering = 10;    //  ± x% deadband
int dead_band_down = 10;        //  ± x% deadband

// Define hybrid variables ...
bool rc_control_down = true;
long timer = 0;
float baro_down_command;
double desired_alt = 15;   // default to 15' altitude


void setup()
{
  Serial.begin(9600);
  Serial.println("*********************    Hybrid Blimp      ***********************");
  Serial.println("The hybrid portion is currently the altitude control - default is");
  Serial.println("to use RC - to switch put the right stick hard-left.  To switch ");
  Serial.println("back, put the stick hard-right");

  // Initialize the barometer (it is important to get calibration values stored on the device).
  if (pressure.begin())
    Serial.println("BMP180 init success");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.
    Serial.println("BMP180 init fail (disconnected?)\n\n");
    Serial.println("Programmed stopped ...");
    while(1)
    {
      digitalWrite(led_pin, HIGH);
      delay(75);
      digitalWrite(led_pin, LOW);
      delay(75);      
    }; // Pause forever.
  }
  baseline = getPressure();   // baseline pressure
  Serial.print("baseline pressure: ");
  Serial.print(baseline);
  Serial.println(" mb");  

  // initialize motor and rc pins ...
  for (int i = 0; i<3; i++)
  {
    pinMode(left_motor[i],  OUTPUT);
    pinMode(right_motor[i], OUTPUT);
    pinMode(down_motor[i],  OUTPUT);
  }
  pinMode(throttle_pin, INPUT);
  pinMode(steering_pin, INPUT);
  pinMode(down_pin,     INPUT);
  pinMode(battery_pin,  INPUT);

  pinMode(led_pin, OUTPUT);

  //  This function calibrates the radio to set the min/max of the pulse-in values - need to figure out a way to do it when desired,
  //  but not every time ...
//    calibrate_radio();

}    // End setup


void loop()    // Main loop
{
  // Determine if altitude is being driven with RC or the barometer, and if it should be switched ...
  aux_command = map(pulseIn(aux_pin, HIGH, 25000), min_pulse_aux, max_pulse_aux, -100, 100);
  if (aux_command < -75)  // assume this means baro-control (this is to the right)
  {
    double alt,P;
    P = getPressure();   // Get a new pressure reading:
    desired_alt = pressure.altitude(P,baseline) / 0.3048;  // set a baseline desired altitude (in feet) based on new pressure ...
    rc_control_down = false;
    for (int i = 1; i < 3; i++)
    {
      digitalWrite(led_pin, HIGH);
      delay(250);
      digitalWrite(led_pin, LOW);
      delay(100);
    }
  }
  else if (aux_command > 75)   // rc control ...
  {
    rc_control_down = true;
    for (int i = 1; i < 3; i++)
    {
      digitalWrite(led_pin, HIGH);
      delay(100);
      digitalWrite(led_pin, LOW);
      delay(250);
    }
  }

  if (rc_control_down == false)
  {
    if ( millis() - timer > 250 )     // check pressure every 1/4 second
    {
      timer = millis();
      double alt,P;
      P = getPressure();   // Get a new pressure reading:
      alt = pressure.altitude(P,baseline) / 0.3048;  // calculate altitude (in feet) based on new pressure ...
      float alt_error = alt - desired_alt;
      float baro_gain = 15;
      //  This control law does nothing if within 2' of desired altitude, and linear proportional response outside of 2'
      baro_down_command = 0;
      if (abs(alt_error) > 2) baro_down_command = alt_error*baro_gain;  // this assumes a command of 0 remains constant, -100 full down, +100 full up
      baro_down_command = constrain(baro_down_command, -100, 100);    // limit command between ±100
//      Serial.print("    desired_alt = ");
//      Serial.print(desired_alt);
//      Serial.print("    alt_error = ");
//      Serial.print(alt_error);
//      Serial.print("    baro_down_command = ");
//      Serial.print(baro_down_command);
//      Serial.println("");
    }
  }
  //  Map input commands to ±100 - this is so -100 = full-speed backwards, +100 = full-speed forward
  throttle_command = map(pulseIn(throttle_pin, HIGH, 25000), min_pulse_throttle, max_pulse_throttle, -100, 100);
  steering_command = map(pulseIn(steering_pin, HIGH, 25000), min_pulse_steering, max_pulse_steering, -100, 100);
  rc_down_command  = map(pulseIn(down_pin,     HIGH, 25000), min_pulse_down,     max_pulse_down,     -100, 100);

  down_command = rc_down_command;
  if (rc_control_down == false) down_command = baro_down_command;



  // Apply dead-band ...
  if (abs(throttle_command) < dead_band_throttle) throttle_command = 0;
  if (abs(steering_command) < dead_band_steering) steering_command = 0;
  if (abs(down_command)     < dead_band_down)     down_command = 0;

  //  Map motor signals to ±100, based on min/max pwm signals (so you can adjust the max speed above)
  //    this also sets -100 to full-speed backwards, and +100 to full-speed forward
  left_motor_signal  = map(throttle_command + 0.5*steering_command, -100, 100, min_pwm_signal_throttle, max_pwm_signal_throttle);
  right_motor_signal = map(throttle_command - 0.5*steering_command, -100, 100, min_pwm_signal_throttle, max_pwm_signal_throttle);
  down_motor_signal  = map(down_command, -100, 100, min_pwm_signal_down, max_pwm_signal_down);

  //  The mapping above still allows the numbers to over-write the limits - this fixes the high 
  //    and low signals to not go over the limits ...
  left_motor_signal  = 0.7 * constrain(left_motor_signal, min_pwm_signal_throttle, max_pwm_signal_throttle);
  right_motor_signal = constrain(right_motor_signal, min_pwm_signal_throttle, max_pwm_signal_throttle);
  down_motor_signal  = constrain(down_motor_signal, min_pwm_signal_down, max_pwm_signal_down);
  //  }

  go(left_motor,  left_motor_signal);
  go(right_motor, right_motor_signal);
  go(down_motor,  down_motor_signal);

}    // end loop



void go(int motor[], int motor_signal)
// Send positive value for motor signal for forward, negative for backwards
{
  if (motor_signal == 0)   // stopped
  { 
    digitalWrite(motor[0], LOW);
  }    
  else if (motor_signal > 0)  // forward
  {
    analogWrite (motor[0], abs(motor_signal));
    digitalWrite(motor[1],HIGH);
    digitalWrite(motor[2],LOW);
  }
  else                        // backward
  {
    analogWrite (motor[0], abs(motor_signal));
    digitalWrite(motor[1],LOW);
    digitalWrite(motor[2],HIGH);
  }
}


bool check_battery(int battery_pin)
{
  float r_high = 100000;
  float r_low = 10000;
  int v_read = analogRead(battery_pin);
  float v_battery = v_read*5.0/1023.0 * (r_high + r_low) / (r_low);
  bool battery_low = false;
  if (v_battery < 6.5) battery_low = true;
  Serial.print("Battery Voltage = ");
  Serial.print(v_battery);
  Serial.print("   ");
  return battery_low;
}

// the initialize_radio function initializes the min/max pulse signals from the various channel inputs
void calibrate_radio()
{
  Serial.println("Calibrating radio - push the sticks every direction");
  long start_time = millis();
  long etime = 0;
  min_pulse_throttle = 1500;
  max_pulse_throttle = 1500;
  min_pulse_steering = 1500;
  max_pulse_steering = 1500;
  min_pulse_down = 1500;
  max_pulse_down = 1500;
  while (etime < 8000)

  {
    throttle_command = pulseIn(throttle_pin, HIGH, 25000);
    steering_command = pulseIn(steering_pin, HIGH, 25000);
    down_command     = pulseIn(down_pin,     HIGH, 25000);

    min_pulse_throttle = min(min_pulse_throttle, throttle_command);
    max_pulse_throttle = max(max_pulse_throttle, throttle_command);
    min_pulse_steering = min(min_pulse_steering, steering_command);
    max_pulse_steering = max(max_pulse_steering, steering_command);
    min_pulse_down     = min(min_pulse_down, down_command);
    max_pulse_down     = max(max_pulse_down, down_command);

    etime = millis() - start_time;
  }
  Serial.println("Calibration Complete ...");
  Serial.println("***********************************************************************************");
  Serial.println("***********************************************************************************");
  Serial.println("***********************************************************************************");
  Serial.print("   min throttle:  ");
  Serial.print(min_pulse_throttle);
  Serial.print("   max throttle:  ");
  Serial.println(max_pulse_throttle);
  Serial.print("   min steering:  ");
  Serial.print(min_pulse_steering);
  Serial.print("   max steering:  ");
  Serial.println(max_pulse_steering);
  Serial.print("   min down:  ");
  Serial.print(min_pulse_down);
  Serial.print("   max down:  ");
  Serial.println(max_pulse_down);

}


double getPressure()
{
  char status;
  double T,P,p0,a;

  // You must first get a temperature measurement to perform a pressure reading.

  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:

    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Use '&T' to provide the address of T to the function.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Use '&P' to provide the address of P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          return(P);
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");
}
