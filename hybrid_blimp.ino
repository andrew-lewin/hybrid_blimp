
// Quentin McRae
// Nov 15, 2013

// initialize barometer stuff ...
// #include <SFE_BMP180.h>
#include <Wire.h>
#include <NewPing.h>

// Initialize other stuff
int left_motor[]  = {3, 2,  4 }; // 3-pins controlling motor thru h-bridge
int right_motor[] = {5, A2, A3}; // 3-pins controlling motor thru h-bridge
int down_motor[]  = {6, 8, 9};   // 3-pins controlling motor thru h-bridge
int left_motor_signal  = 0;      // Initial motor signal (initiallly stopped until there is a reason to move)
int right_motor_signal = 0;      // Initial motor signal (initiallly stopped until there is a reason to move)
int down_motor_signal  = 0;      // Initial motor signal (initiallly stopped until there is a reason to move)
int aux_command;                 // < -50 - switch to baro, > +50, which to RC
int throttle_command;            // -100 is full-speed backward, +100 is full-speed forward
int steering_command;            // +100 is full right turn, -100 is full left turn, 0 is straight
int rc_down_command;             // +100 is full speed, 0 is off
int down_command;                // +100 is full speed, 0 is off
int aux_pin = 10;                // Pin that the aux input comes in on (used to determine RC vs. baro altitude control)
int throttle_pin = 7;            // Pin that the throttle input comes in on
int steering_pin = 11;           // Pin that the steering input comes in on
int down_pin = 12;               // Pin that the altitude-fan input comes in on
int battery_pin = A1;            // Pin to read the battery voltage
int led_pin = 13;

// Initialize min and max radio inputs ...
int min_pulse_throttle = 1200;   // These values are to set the full left/right and forward/reverse pulse values from the radio
int max_pulse_throttle = 1940;   // These should probably be found by some sort of startup calibration
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

#define TRIGGER_PIN  12  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     11  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.


void setup()
{
  Serial.begin(9600);
  
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
  
  // initialize sonar
  Serial.begin(9600); // Open serial monitor at 115200 baud to see ping results.

  //  This function calibrates the radio to set the min/max of the pulse-in values - need to figure out a way to do it when desired,
  //  but not every time ...
//    calibrate_radio();

}    // End setup

// Main Loop
void loop() {
  check_ping();

  Serial.println("Taking RC input");
  
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

  go(left_motor,  left_motor_signal);
  go(right_motor, right_motor_signal);
  go(down_motor,  down_motor_signal);

}    // end loop

void check_ping() {
  if (!approaching_object()) return; // we can still be in control with the RC if we aren't approaching the object even if in dangerous distance
  cut_rc_and_force_backward();
  check_ping();
}

float get_distance() {
  delay(20);
  unsigned int ping_uS = sonar.ping(); // Send the ping. Get ping time in micro seconds (uS)
  float calculated_distance_cm = ping_uS / US_ROUNDTRIP_CM; // Convert ping time to distance in cm
  return calculated_distance_cm;
}

void cut_rc_and_force_backward() {
  throttle_command = -25; // 25% speed backwards
  steering_command = 0;    // don't turn
  rc_down_command  = 0;    // don't change height

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
  
  go(left_motor,  left_motor_signal);
  go(right_motor, right_motor_signal);
  go(down_motor,  down_motor_signal);

  while(true) {
    if (approaching_object()){
      Serial.println("Mayday! Mayday!");
    }else {
      break;
    }
  }
}

boolean approaching_object(){
  float start_distance = get_distance(); // Save initial distance
  if (start_distance > 150) return false; // We don't care if we are more than 150 CM out.
  for(int i = 0; i < 3; i++){
    get_distance(); // We want one one distance and the 4th one after that
  }
  float end_distance = get_distance(); // Save end distance
  return (end_distance - start_distance) < 0; // Return true if the end distance is less than the start distance, meaning we are appraching
}

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

}
