#include <PID_v1.h>
#include <movingAvg.h>
#include <Servo.h>

class RoverArmMotor{

    public:

        // Our motors use two different ESC's with different control schemes
        #define CYTRON 0
        #define BLUE_ROBOTICS 1

        // ADC values representing 359 and 0 degrees respectively
        #define MAX_ADC_VALUE 3850
        #define MIN_ADC_VALUE 200

        #define FWD 1
        #define REV -1

        RoverArmMotor(int pwm_pin, int encoder_pin, int esc_type, double minimum_angle, 
                      double maximum_angle, int dir_pin);

        // Setters for various tunable parameters of our motors
        void setAggressiveCoefficients(double P, double I, double D);
        void setRegularCoefficients(double P, double I, double D);
        void setRetuningGapLimit(int gap);
        void setAngleLimits(double lowest_angle, double highest_angle);
        bool setMultiplierBool(bool mult, int value); 
        void newSetpoint(double angle);

        void setPIDOutputLimits(double lower_end, double upper_end);
        void setMovingAverageWindowSize(int size);

        float getCurrentAngle();
        double getSetpoint();

        void begin(double aggP, double aggI, double aggD, double regP, double regI, double regD);
        void tick();

    private:
        // Default to open loop, will need to enter the coefficients to begin
        PID internalPIDInstance;
        movingAvg internalAveragerInstance;
        Servo internalServoInstance;

        double aggressiveKp, aggressiveKi, aggressiveKd, regularKp, regularKi, regularKd;
        int pwm, dir, encoder;
        int movingAverageWindowSize;
        double lowestAngle, highestAngle;
        int escType;
        int adcResult;
        double currentAngle, lastAngle;
        bool wrist_waist; 
        int multiplier;
        double input;
        double output;
        double setpoint;
        int actuationState;

        enum ActuationStates
        {
            FIRST_ROTATION_REGION,
            SECOND_ROTATION_REGION,
            RATIO_IS_ONE
        };

        double mapFloat(float x, float in_min, float in_max, float out_min, float out_max);
};

