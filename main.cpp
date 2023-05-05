#include <chrono>
#include <mbed.h>
#include "EncoderCounter.h"
#include "PM2_Drivers.h"
#include "PinNames.h"

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

const float k_gear2 = 156.0f / 78.125f;

const float distanceFromStart = 0.0f;
//state variables
bool execute_main = false;
bool reset_all = false;

//Blue User Button on nucleo board
DebounceIn user_button(PC_13); //Creates interrupt object to evaluate user button.
void user_button_pressed();

/*
DebounceIn small_button_front(PB_2);
void small_button_front_pressed();
int buttonFrontPressed = 0;


DebounceIn small_button_back(PC_9);
void small_button_back_pressed();
int buttonBackPressed = 0;

*/
const int TREADSTER_STATE_INIT     = 0;
const int TREADSTER_STATE_FORWARD  = 1;
const int TREADSTER_STATE_PUSHUP   = 2;
const int TREADSTER_STATE_FORWARD_MINI = 3;
const int TREADSTER_STATE_CLIMBING = 4;
const int TREADSTER_STATE_FORWARD_MINI2 = 5;
const int TREADSTER_STATE_BACKWARD = 6;
const int TREADSTER_STATE_FORWARD_MINI3 = 7;
const int TREADSTER_STATE_STOP     = 8;
const int TREADSTER_STATE_FORWARD_TINY = 9;




int treadster_state_actual = TREADSTER_STATE_INIT;

int counter = 0;
int stop = 0;

float start_arms_rotation = 0.0f;
// led on nucleoboard, for task informations.
DigitalOut board_led(LED1);


// main sequence
int main()
{   
    //SETUP _______________________________________________________________________
    //The following code will be executed once after boot.
    //Treadster states

    printf("Setup\n");
    //attach user_button_pressed to the user button, so that it executes when pressed on falling edge.
    user_button.fall(&user_button_pressed);


   // small_button_front.fall(&small_button_front_pressed);
   // small_button_back.fall(&small_button_back_pressed);

    //main loop gets executed every 50 ms.
    const int main_task_period_ms = 20;
    Timer main_task_timer;

    board_led.write(0);
 
    //Create an DigitalIn Object for the mechanical button. Detects front wall.
 
    /*
    DigitalIn small_button_front(PB_2);
    DigitalIn small_button_back(PC_8);
    small_button_back.mode(PullDown);
    small_button_back.mode(PullUp);
    */
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
    SpeedController speedController_left(counts_per_turn, kn, max_voltage, pwm_left, M2);
    SpeedController speedController_right(counts_per_turn, kn, max_voltage, pwm_right, M3);

    
    PositionController posController_arms(counts_per_turn * k_gear2, kn / k_gear2, max_voltage, pwm_arms, M1);
    posController_arms.setSpeedCntrlGain(0.15 * k_gear2);
    posController_arms.setMaxVelocityRPS(max_speed_rps);


    main_task_timer.start();
    printf("Setup Complete \n");
    // END OF SETUP ___________________________________________________________________________________

    //runs forever
    while(true) {
        main_task_timer.reset();

        /*
        if (small_button_back.read()) {
            printf("back button pressed\n");
        }
        
        if (small_button_front.read()){
            printf("front button pressed");
        }
        */


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
                    speedController_left.setDesiredSpeedRPS(1.5f);
                    speedController_right.setDesiredSpeedRPS(-1.5f);
                    posController_arms.setDesiredRotation(0.0f);


                    if (counter <= 260) {  //140
                    counter = counter + 1;
                    }
                    else {
                    counter = 0;
                     treadster_state_actual = TREADSTER_STATE_PUSHUP;     
                    }
                    //printf("%f", start_arms_rotation);
                    
                    break;
                

                case TREADSTER_STATE_PUSHUP:
    
                    speedController_left.setDesiredSpeedRPS(0.0f);
                    speedController_right.setDesiredSpeedRPS(0.0f);

                    posController_arms.setSpeedCntrlGain(kp * k_gear);
                    posController_arms.setSpeedCntrlGain(0.4f);

                    posController_arms.setDesiredRotation(3.5f);
                    printf("%f \n", posController_arms.getRotation());

                    if (posController_arms.getRotation() >= 3.3f)
                        treadster_state_actual = TREADSTER_STATE_FORWARD_MINI;

                    break;

                case TREADSTER_STATE_FORWARD_MINI:

                    printf(" Going forward ");
                    enable_motors = 1;
                    speedController_left.setDesiredSpeedRPS(1.0f);
                    speedController_right.setDesiredSpeedRPS(-1.0f);
                    posController_arms.setDesiredRotation(4.0f);

                    if (counter <= 150) { //60
                    counter = counter + 1;
                    }
                    else {
                    counter = 0;
                     treadster_state_actual = TREADSTER_STATE_CLIMBING;     
                    }
                    //printf("%f", start_arms_rotation);
                    
                    break;
                
                case TREADSTER_STATE_CLIMBING:
                    speedController_left.setDesiredSpeedRPS(0.0f);
                    speedController_right.setDesiredSpeedRPS(-0.0f);
                    
                    posController_arms.setSpeedCntrlGain(kp * k_gear);
                    posController_arms.setSpeedCntrlGain(0.6f);

                    posController_arms.setDesiredRotation(-13.0f);

                    if (posController_arms.getRotation() <= -12.8f)
                        treadster_state_actual = TREADSTER_STATE_FORWARD_TINY;

                    break;

                case TREADSTER_STATE_FORWARD_TINY:

                    
                    speedController_left.setDesiredSpeedRPS(0.8f);
                    speedController_right.setDesiredSpeedRPS(-0.8f);

                    if (counter <= 150) {
                    counter = counter + 1;
                    }
                    else {
                    counter = 0;
                     treadster_state_actual = TREADSTER_STATE_FORWARD_MINI2;     
                    } 
                    break;                   


                case TREADSTER_STATE_FORWARD_MINI2:

                    printf(" Going forward ");
                    enable_motors = 1;



                    if (stop == 0) {
                        speedController_left.setDesiredSpeedRPS(0.8f);
                        speedController_right.setDesiredSpeedRPS(-0.8f); 
                    }


                    posController_arms.setSpeedCntrlGain(0.4f);
                    posController_arms.setDesiredRotation(0.0f);

                    if (counter <= 85  && stop == 0) {  //100
                    counter++;
                    }
                    else if (counter <= 105) {
                    stop = 1;
                    counter++;
                    speedController_left.setDesiredSpeedRPS(0.0f);
                    speedController_right.setDesiredSpeedRPS(0.0f);
                    } else if (counter <= 125) {
                        speedController_left.setDesiredSpeedRPS(0.4f);
                        speedController_right.setDesiredSpeedRPS(0.4f);
                        counter++;
                    } else if (counter > 125) {
                        speedController_left.setDesiredSpeedRPS(0.0f);
                        speedController_right.setDesiredSpeedRPS(0.0f);
                    } 

                 

                    if (posController_arms.getRotation() >= -0.2f) {
                        counter = 0;
                        treadster_state_actual = TREADSTER_STATE_BACKWARD;
                    }
                    //printf("%f", start_arms_rotation);
                    
                    break;


                case TREADSTER_STATE_BACKWARD:
                   // posController_left.setDesiredRotation(-distanceFromStart);
                    printf(" Going Backwards ");
                    speedController_left.setDesiredSpeedRPS(-1.0f);
                    speedController_right.setDesiredSpeedRPS(1.0f);

                    if (counter <= 50) { //60
                    counter = counter + 1;
                    }
                    else {
                    counter = 0;
                     treadster_state_actual = TREADSTER_STATE_FORWARD_MINI3;     
                    }
                    
                    break;



                case TREADSTER_STATE_FORWARD_MINI3:

                    printf(" Going forward ");
                    enable_motors = 1;
                    speedController_left.setDesiredSpeedRPS(2.0f);
                    speedController_right.setDesiredSpeedRPS(-2.0f);


                    if (counter < 162) { //300
                    counter = counter + 1;
                    }
                    else {
                    counter = 0;
                     treadster_state_actual = TREADSTER_STATE_STOP;
                    }
                    //printf("%f", start_arms_rotation);
                    
                    break;

                case TREADSTER_STATE_STOP:
                    printf(" Stopping ");
                    speedController_left.setDesiredSpeedRPS(0.0f);
                    speedController_right.setDesiredSpeedRPS(0.0f);
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

    treadster_state_actual = (treadster_state_actual == 9) ? 0 : (treadster_state_actual + 1);

}

void test_button_pressed() {

}

void mechanical_button_pressed() {

    treadster_state_actual = (treadster_state_actual == 9) ? 0 : (treadster_state_actual + 1);
} 

/*
void small_button_back_pressed() {
    printf("Button back pressed");
    buttonBackPressed = 1;
}

void small_button_front_pressed() {
    printf("Button Front Pressed");
    buttonFrontPressed = 1;
}
*/