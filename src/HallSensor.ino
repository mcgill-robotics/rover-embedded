/**
 * Define type of rotations
 * 1. Clockwise with a 1
 * 2. Counterclockwise with a -1
 */
#define clockW 1
#define counterCW -1

/**
 * Set initial pin values
 * Assign three individual values for each pin corresponding sensor
 * Get a boolean value of the initial state of each sensor
 * Assign a value of number of hall rotations for RPM readings
 */
bool hallA_value = digitalRead(2);
bool hallB_value = digitalRead(3);
bool hallC_value = digitalRead(4);
float hall_th = 100.0;

/**
 * Assign and/or initialize inital values and variables
 * 1. direction of BLDC rotation
 * 2. Pulse count
 * 3. Start time of current interrupt
 * 4. Previous time of interrupt
 * 5. Pulse time for Sensor A
 * 6. Pulse time for Sensor B
 * 7. Pulse time for Sensor C
 * 8. Avg pulse time
 * 9. Pulses per minute 
 * 10. Revolutions per minute
 * 11. Angular velocity
 */
int dir = 1;
int pulses;
float sTime;
float pTime;
float pulseA;
float pulseB;
float pulseC;
float avgPulse;
float PPM;
float RPM;
float aVelocity;

void setup() {
  // put your setup code here, to run once:
  // Set digital pins as inputs
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  
  // Set digital pins with interrupts to respond to rising and falling edges
  attachInterrupt(digitalPinToInterrupt(2), HallSensor_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), HallSensor_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), HallSensor_C, CHANGE);
  
  // Set up serial communication at a baud rate = 9600
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if ((millis() - pTime) > 600)  RPM = 0;
  Serial.println("RPM: \n");
  Serial.println(RPM);
  Serial.println("Angular Velocity: \n"); 
  Serial.println(aVelocity);
}

void HallSensor_A() {
  // Set start time
  sTime = millis();
  // Read current B and C values
  hallA_value = digitalRead(2);
  hallC_value = digitalRead(4);
  // Determine direction with if statement incorportated
  dir = (hallB_value == hallC_value) ? clockW : counterCW;
  // Incremment pulse count
  pulses = pulses + (1 * dir);
  // Calculate current time between pulses for sensor A
  pulseA = sTime - pTime;
  // Calculate average pulse time
  avgPulse = ((pulseC + pulseB + pulseA) / 3);
  // Compute pulser per minute
  PPM = (1000 / avgPulse) * 60;
  // Calculate revs per minute
  RPM = PPM / 90;
  // Calculate angular velocity
  aVelocity = RPM * 6;
  // Store start time for next interrupt
  pTime = sTime;
}

void HallSensor_B() {
 // Set start time
  sTime = millis();
  // Read current B and C values
  hallA_value = digitalRead(2);
  hallB_value = digitalRead(3);
  // Determine direction with if statement incorportated
  dir = (hallA_value == hallB_value) ? clockW : counterCW;
  // Incremment pulse count
  pulses = pulses + (1 * dir);
  // Calculate current time between pulses for sensor A
  pulseA = sTime - pTime;
  // Calculate average pulse time
  avgPulse = ((pulseC + pulseB + pulseA) / 3);
  // Compute pulser per minute
  PPM = (1000 / avgPulse) * 60;
  // Calculate revs per minute
  RPM = PPM / 90;
  // Calculate angular velocity
  aVelocity = RPM * 6;
  // Store start time for next interrupt
  pTime = sTime;
}

void HallSensor_C() {
  // Set start time
  sTime = millis();
  // Read current B and C values
  hallB_value = digitalRead(3);
  hallC_value = digitalRead(4);
  // Determine direction with if statement incorportated
  dir = (hallB_value == hallC_value) ? clockW : counterCW;
  // Incremment pulse count
  pulses = pulses + (1 * dir);
  // Calculate current time between pulses for sensor A
  pulseA = sTime - pTime;
  // Calculate average pulse time
  avgPulse = ((pulseC + pulseB + pulseA) / 3);
  // Compute pulser per minute
  PPM = (1000 / avgPulse) * 60;
  // Calculate revs per minute
  RPM = PPM / 90;
  // Calculate angular velocity
  aVelocity = RPM * 6;
  // Store start time for next interrupt
  pTime = sTime;
}
