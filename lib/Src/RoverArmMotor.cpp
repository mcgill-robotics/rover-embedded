#include <Arduino.h>
#include <RoverArmMotor.h>



// TODO: Test this class with the old code, remember to create backup beforehand!
// I'm very suspicious of the way I handled user defined pointers...

// The motor will not move until begin() is called. You also need to run setAngleLimits().
RoverArmMotor::RoverArmMotor(int pwm_pin, int encoder_pin, int esc_type, double minimum_angle, double maximum_angle, int dir_pin)
                :internalPIDInstance(&input, &output, &setpoint, regularKp, regularKi, regularKd, DIRECT)
                ,internalAveragerInstance(15){

    encoder = encoder_pin;
    lowestAngle = minimum_angle;
    highestAngle = maximum_angle;
    pwm = pwm_pin;
    dir = dir_pin;
    escType = esc_type;

    // Initialize these two such that it's impossible to respect them. This forces
    // the main function to also run setAngleLimits(), or else the motor won't move
    minAngle = -1;
    maxAngle = -10;
    
}

void RoverArmMotor::begin(double aggP, double aggI, double aggD, double regP, double regI, double regD){

    // Initialize given pins
    pinMode(encoder, INPUT);
    pinMode(pwm, OUTPUT);

    if(escType == CYTRON){
        pinMode(dir, OUTPUT);
        // Allow negative outputs, the sign will be interpreted as
        // the direction pin
        internalPIDInstance.SetOutputLimits(-255, 255);
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

    //initialize the gear ratio to 1. 
    gearRatio = 1;

}

// Needs to be called in each loop
void RoverArmMotor::tick(){

    // Get current angle
    adcResult = internalAveragerInstance.reading(analogRead(encoder));
    currentAngle = mapFloat((float) adcResult, MAX_ADC_VALUE, MIN_ADC_VALUE, 359.0f, 0.0f);
    input = currentAngle;

      // Measurement deadband - ignore sub-degree noise
    if(abs(currentAngle - lastAngle) < 1.0){
        currentAngle = lastAngle;
    }

    currentAngle /= gearRatio;

    if (abs(gap) < 10){
        internalPIDInstance.SetTunings(regularKp, regularKi, regularKd);
    }else{
        internalPIDInstance.SetTunings(aggressiveKp, aggressiveKi, aggressiveKd);
    }

    // Compute the next value
    internalPIDInstance.Compute();

    // Check maximum and minimum angle limits to make sure the arm isn't breaking shit
    if(currentAngle <= maxAngle && currentAngle >= minAngle){

        // Interpret output data based on the ESC type defined in constructor
        if(escType == CYTRON){

            // Interpret sign of the error signal as the direction pin value
            (gap > 0) ? digitalWrite(dir, LOW) : digitalWrite(dir, HIGH);

            // Write to PWM pin
            analogWrite(pwm, output); 

        }else if(escType == BLUE_ROBOTICS){
            // This one is more straightforward since we already defined the output range
            // from 1100us to 1900us
            internalServoInstance.writeMicroseconds(output);
        }

    }

    lastAngle = currentAngle;
    
}

void RoverArmMotor::setGearRatio(double value){
    gearRatio = value; 
}

double RoverArmMotor::getSetpoint(){
    return setpoint;
}

void RoverArmMotor::newSetpoint(double angl){
    setpoint = angl;
}

void RoverArmMotor::setAngleLimits(double min, double max){
    minAngle = min;
    maxAngle = max;
}

float RoverArmMotor::getCurrentAngle(){
    return currentAngle;
}

double RoverArmMotor::mapFloat(float x, float in_min, float in_max, float out_min, float out_max){
    return (double) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}