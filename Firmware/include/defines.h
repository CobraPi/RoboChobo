#define DEBUG 1

#define BAUD_RATE 115200

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
#define MAX_SPEED 4000.0f
#define MIN_SPEED 1

#define REFRESH_RATE 100.0f
#define ACC_RATE 100.0f
#define GYRO_RATE 100.8f

#define ANGLE_P 19.0
#define ANGLE_I 5.0
#define ANGLE_D 30.0

#define SPEED_P 0.0165f
#define SPEED_I 0.0f
#define SPEED_D 0.00425f