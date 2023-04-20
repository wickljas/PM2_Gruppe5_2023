#include <chrono>
#include <mbed.h>
#include "PM2_Drivers.h"

// PIN Definitions
#define pinM1 PB_13
#define pinM2 PA_9
#define pinM3 PA_10
#define pinEnableMotors PB_15
#define pinButton PC_5

//Motor Values
const float max_voltage = 12.0f;               // define maximum voltage of battery packs, adjust this to 6.0f V if you only use one batterypack
const float counts_per_turn = 20.0f * 78.125f; // define counts per turn at gearbox end: counts/turn * gearratio
const float kn = 180.0f / 12.0f;               // define motor constant in RPM/V
const float k_gear = 100.0f / 78.125f;         // define additional ratio in case you are using a dc motor with a different gear box, e.g. 100:1
const float kp = 0.2f;                         // define custom kp, this is the default speed controller gain for gear box 78.125:1
const float max_speed_rps = 1.0f;

const float k_gear2 = 100.0f / 156.0f;
const float counts_per_turn2 = 20.0f * 156.0f;

const float distanceFromStart = 1.0f;
//state variables
bool execute_main = false;
bool reset_all = false;

//Blue User Button on nucleo board
DebounceIn user_button(PC_13); //Creates interrupt object to evaluate user button.
void user_button_pressed();

DebounceIn test_button(D15);
void test_button_pressed();
bool testactive = false;

const int TREADSTER_STATE_INIT     = 0;
const int TREADSTER_STATE_FORWARD  = 1;
const int TREADSTER_STATE_BACKWARD = 2;
const int TREADSTER_STATE_STOP     = 3;
const int TREADSTER_STATE_CLIMBING = 4;
const int TREADSTER_STATE_PUSHUP   = 5;
const int TREADSTER_STATE_ROTATING = 6;
int treadster_state_actual = TREADSTER_STATE_INIT;


float start_arms_rotation = 0.0f;
// led on nucleoboard, for task informations.
DigitalOut board_led(LED1);


// main sequence
int main()
{   
    //SETUP _____________________________________________________________________________________________________________________________________________________________________________________________________________________
    //The following code will be executed once after boot.
    //Treadster states

    printf("Setup\n");
    //attach user_button_pressed to the user button, so that it executes when pressed on falling edge.
    user_button.fall(&user_button_pressed);
    test_button.fall(&test_button_pressed);

    //main loop gets executed every 50 ms.
    const int main_task_period_ms = 50;
    Timer main_task_timer;

    board_led.write(0);


    
    //Create an DigitalIn Object for the mechanical button, which triggers when Treadster crosses the wall.
    DigitalIn mechanical_button(pinButton);
    mechanical_button.mode(PullUp);

    // DigitalIn test_button(D15);
    //test_button.mode(PullDown);

    
    //Create digitalout object to enable DC-Motors
    DigitalOut enable_motors(pinEnableMotors);

    FastPWM pwm_left(pinM2);  //Drives Left Track
    FastPWM pwm_right(pinM3);  //Drives Right Track
    FastPWM pwm_arms(pinM1);  //Drives Arms

    //Create Encoder Objects of the DC Motors.
    EncoderCounter M1(PA_6, PC_7);
    EncoderCounter M2(PB_6, PB_7);
    EncoderCounter M3(PA_0, PA_1);

    //Create Speed Controller and Position Controller Objects for the DC motors. 
    //Left and Right Motors should have both a speed and a position configuration, since they need to switch between those two different control algorithms.
    SpeedController speedController_left(counts_per_turn, kn, max_voltage, pwm_left, M2);
    SpeedController speedController_right(counts_per_turn, kn, max_voltage, pwm_right, M3);

    
    //PositionController posController_left(counts_per_turn * k_gear, kn / k_gear, max_voltage, pwm_left, encoder_left);
    //PositionController posController_right(counts_per_turn * k_gear, kn / k_gear, max_voltage, pwm_right, encoder_right);
    

    PositionController posController_arms(counts_per_turn2 * k_gear2, kn / k_gear2, max_voltage, pwm_arms, M1);
    posController_arms.setSpeedCntrlGain(0.05 * k_gear2);
    posController_arms.setMaxVelocityRPS(max_speed_rps);

    //posController_left.setSpeedCntrlGain(kp * k_gear);   // adjust internal speed controller gain, this is just an example
    //posController_left.setMaxVelocityRPS(max_speed_rps); //
   

    main_task_timer.start();
    printf("Setup Complete \n");
    // END OF SETUP _____________________________________________________________________________________________________________________________________________________________________________________________________________________________________________________

    //runs forever
    while(true) {
        main_task_timer.reset();

        
/*      
        if (!test_button.read()) {
            test_button_pressed();
        }
*/
        if (testactive) {
            printf("Current Treadster State: %d\n", treadster_state_actual);
            testactive = !testactive;
        }
        //Starts main Sequence of Treadster.
        if (execute_main) {
           
            switch (treadster_state_actual) {
                //Insert main code for the Treadster here.
                case TREADSTER_STATE_INIT:
                    speedController_left.setDesiredSpeedRPS(0.0f);
                    speedController_right.setDesiredSpeedRPS(0.0f);
                    start_arms_rotation = posController_arms.getRotation();
                    break;

                case TREADSTER_STATE_FORWARD:
                   // posController_left.setDesiredRotation(distanceFromStart);
                    printf(" Going forward ");
                    enable_motors = 1;
                    speedController_left.setDesiredSpeedRPS(1.0f);
                    speedController_right.setDesiredSpeedRPS(-1.0f);
                    posController_arms.setDesiredRotation(0.0f);
                    //printf("%f", start_arms_rotation);
                    
                    break;
                
                case TREADSTER_STATE_BACKWARD:
                   // posController_left.setDesiredRotation(-distanceFromStart);
                    printf(" Going Backwards ");
                    speedController_left.setDesiredSpeedRPS(-1.0f);
                    speedController_right.setDesiredSpeedRPS(1.0f);
                    break;

                case TREADSTER_STATE_STOP:
                    printf(" Stopping ");
                    speedController_left.setDesiredSpeedRPS(0.0f);
                    speedController_right.setDesiredSpeedRPS(0.0f);
                    break;
                
                case TREADSTER_STATE_CLIMBING:
                    
                    break;
                
                case TREADSTER_STATE_PUSHUP:
    
                    speedController_left.setDesiredSpeedRPS(0.0f);
                    speedController_right.setDesiredSpeedRPS(0.0f);

                    posController_arms.setSpeedCntrlGain(kp * k_gear);
                    posController_arms.setSpeedCntrlGain(0.5f);

                    posController_arms.setDesiredRotation(-0.5f);
                    printf("%f \n", posController_arms.getRotation());

                    break;
                
                case TREADSTER_STATE_ROTATING:
                    speedController_left.setDesiredSpeedRPS(0.5f);
                    speedController_right.setDesiredSpeedRPS(-0.5f);
                    
                    posController_arms.setMaxVelocityRPS(0.5f);
                    posController_arms.setDesiredRotation(0.5f);
                    break;

                default: break;
            }
        } else if (reset_all) {
            reset_all = false;
            //Reset all Variables here.
        }

        int main_task_elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(main_task_timer.elapsed_time()).count();
        thread_sleep_for(main_task_period_ms - main_task_elapsed_time_ms);

    }
}

void user_button_pressed() {
    //Toggle execute_main state
    execute_main = true;
    //printf("User Button Pressed\n");

    if (execute_main) {
        board_led = 1;
    } else {
        board_led = 0;
    }
    //printf("Current Treadster State: %d\n", treadster_state_actual);

    //Reset all the variables that may have changed in the last run.
     if (execute_main)
        reset_all = true;

    treadster_state_actual = (treadster_state_actual == 6) ? 0 : (treadster_state_actual + 1);

}

//test function to evaluate the different states of the treadster, using a mechanical button.
void test_button_pressed() {
    //execute_main = true;

    testactive = true;
    board_led = true;
    board_led = false;

    treadster_state_actual = (treadster_state_actual == 6) ? 0 : (treadster_state_actual + 1);
    //printf("Current Treadster State: %d\n", treadster_state_actual);

   
}