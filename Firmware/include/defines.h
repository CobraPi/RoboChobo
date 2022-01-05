#define DEBUG 1

#define BAUD_RATE 9600

#define L_STEP_PIN 2
#define L_DIR_PIN  5

#define R_STEP_PIN 4
#define R_DIR_PIN  7
/*
   Used for pulse generation
   1/1 step: 62
   1/2 step: 31
   1/4 step: 16
*/
#define MIN_TICKS 16

#define MAX_ANGLE  40
#define CG 0
#define MAX_SPEED 1000.0f
#define MIN_SPEED 1