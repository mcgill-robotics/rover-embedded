#include <Arduino.h>
#include <RoverArmMotor.h>



// TODO: Test this class with the old code, remember to create backup beforehand!
// I'm very suspicious of the way I handled user defined pointers...

// The motor will not move until begin() is called!
RoverArmMotor::RoverArmMotor(int pwm_pin, int encoder_pin, int esc_type, double minimum_angle, double maximum_angle, int dir_pin, int brake_pin)
                :internalPIDInstance(&input, &output, &setpoint, regularKp, regularKi, regularKd, DIRECT)
                ,internalAveragerInstance(15){

    encoder = encoder_pin;
    lowestAngle = minimum_angle;
    highestAngle = maximum_angle;
    pwm = pwm_pin;
    dir = dir_pin;
    escType = esc_type;
    brake = brake_pin;
    
}

void RoverArmMotor::begin(double aggP, double aggI, double aggD, double regP, double regI, double regD){

    // Initialize given pins
    pinMode(encoder, INPUT);
    pinMode(pwm, OUTPUT);

    if(escType == CYTRON){
        pinMode(dir, OUTPUT);
        // Allow negative outputs, the sign will be interpreted as
        // the direction pin
        internalPIDInstance.SetOutputLimits(-50, 50);
    }
    else if(escType == BLUE_ROBOTICS){
         // BlueRobotics ESC uses a servo-like control scheme where
        // 1100us is full speed reverse and 1900us is full speed forward
        internalPIDInstance.SetOutputLimits(1100, 1900);
        internalServoInstance.attach(pwm);
    }
    
    // Initialize moving averager
    internalAveragerInstance.begin();

    // Set to auto
    internalPIDInstance.SetMode(AUTOMATIC);

    // Get current location and set it as setpoint. Essential to prevent jerkiness
    // as the microcontroller initializes.
    adcResult = internalAveragerInstance.reading(analogRead(encoder));
    currentAngle = mapFloat((float) adcResult, MAX_ADC_VALUE, MIN_ADC_VALUE, 359.0f, 0.0f);
    setpoint = currentAngle;

    // Set tuning params
    regularKp = regP;
    regularKi = regI;
    regularKd = regD;
    aggressiveKp = aggP;
    aggressiveKi = aggI;
    aggressiveKd = aggD;

    internalPIDInstance.SetTunings(regularKp, regularKi, regularKd);
    if(brake)  engageBrake();
    //initialize the multiplier bool to false and the multiplier to 1. 
    wrist_waist = false; 
    //multiplier = 1;
    gearRatio = 1;

}

int positive_rezeros = 0;
double real_angle = 0;

// Needs to be called in each loop
void RoverArmMotor::tick(){

    // Get current angle
    adcResult = internalAveragerInstance.reading(analogRead(encoder));
    currentAngle = mapFloat((float) adcResult, MAX_ADC_VALUE, MIN_ADC_VALUE, 359.0f, 0.0f);
    
      // Measurement deadband - ignore sub-degree noise
    if(abs(currentAngle - lastAngle) < 1.0){
        currentAngle = lastAngle;
    }

    input = currentAngle;

    // Compute distance, retune PID if necessary. Less aggressive tuning params for small errors
    // Find the shortest from the current position to the set point
    double gap;

    if(wrist_waist){
        (abs(setpoint-input) < abs((setpoint + 360.0f)-input)) ? gap = setpoint - input : gap = (setpoint + 360.0f) - input; 
    }else{
        gap = setpoint - input;
    }

    // If we are outside of angle bounds, make a setpoint intervention to bring the shaft to the midpoint
    if(input <= lowestAngle || input >= highestAngle){
        setpoint = gearRatio * (lowestAngle + highestAngle) / 2 ;
    }

    // Tone down P and I as the motor hones onto position
    if (abs(gap) < 10){
        internalPIDInstance.SetTunings(regularKp, regularKi, regularKd);
    }else{
        internalPIDInstance.SetTunings(aggressiveKp, aggressiveKi, aggressiveKd);
    }

    // Compute the next value
    internalPIDInstance.Compute();

    // Refuse to go to setpoint if it is outside of boundaries
    // if(setpoint <= lowestAngle || setpoint >= highestAngle){
    //     output = 0;
    // }

    // Make sure we aren't snapping our tendons - move back a little bit if we are
    // if(currentAngle >= (highestAngle - 2) && currentAngle <= (lowestAngle + 2)) output = 0.0;

    // Interpret output data based on the ESC type defined in constructor
    if(escType == CYTRON){

        // Interpret sign of the error signal as the direction pin value
        (gap > 0) ? digitalWrite(dir, HIGH) : digitalWrite(dir, LOW);

        // Write to PWM pin
        analogWrite(pwm, output); 

    }else if(escType == BLUE_ROBOTICS){
        // This one is more straightforward since we already defined the output range
        // from 1100us to 1900us
        internalServoInstance.writeMicroseconds(output);
    }

    lastAngle = currentAngle;
    
}

bool RoverArmMotor::setMultiplierBool(bool mult, double ratio){
    wrist_waist = mult; 
    gearRatio = ratio; 
    //a bit redundant but just a sanity check of a second getter method
    if(getRatio() == ratio) return true; 
    return false; 
}

// For display purposes
double RoverArmMotor::getSetpoint(){
    return setpoint / gearRatio;
}

bool RoverArmMotor::newSetpoint(double angl){
    double setpoint_test = angl * gearRatio;
    if(setpoint_test > lowestAngle && setpoint_test < highestAngle){
        setpoint = setpoint_test;
        return true;
    }else{
        return false;
    }
}

int RoverArmMotor::getDirection(){
    return (digitalRead(dir) == HIGH) ? FWD : REV;
}

// void RoverArmMotor::setGearRatio(double ratio){
//     gearRatio = ratio;
// }

void RoverArmMotor::setAngleLimits(double lowest, double highest){
    lowestAngle = lowest * gearRatio;
    highestAngle = highest * gearRatio;
}

void RoverArmMotor::disengageBrake(){
    if(brake != 0){
      digitalWrite(brake, LOW);  
    }
}

void RoverArmMotor::engageBrake(){
    if(brake != 0){
       digitalWrite(brake, HIGH); 
    }
}

double RoverArmMotor::getCurrentAngle(){
    return currentAngle / gearRatio;
}

double RoverArmMotor::getCurrentOutput(){
    return output;
}

double RoverArmMotor::mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
    return (double) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

double RoverArmMotor::getRatio(){
    return gearRatio;
}

void RoverArmMotor::WatchdogISR(){
    // Get current angle

    // Set setpoint to that angle
}