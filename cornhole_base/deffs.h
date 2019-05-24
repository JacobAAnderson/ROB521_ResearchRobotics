// Constants -----------------------------
#define WHEEL_ENCODER_INC  0.00225      // Drive wheel encoder step   [rad]
#define ARM_ENCODER_INC    0.0123       // Arm encoder step           [rad]   0.00744 
#define DRIVE_WHEEL_RADIUS 0.076        // Radius of the drive wheel  [m]
#define ARM_RADIUS         0.254
#define MAX_ALPHA          0.25         // Max Motor acceleration     [rad/s^2]
#define MAX_DRIVE_OMEGA    1.9          // Max Drving motor velocity  [rad/s]
#define MAX_ARM_OMEGA     35.6047167    // Max Thowing motor velocity [rad/s]
#define ARM_GEAR_RATIO     0.941176     // 16/17 Gear Ratio
#define RPM_RADS           0.10472
// I/O Pins-------------------------------

#define PIN_SERVO 4

#define PIN_ARM_DIR  9
#define PIN_ARM_PWM  8

#define PIN_LEFT_DIR  10
#define PIN_LEFT_PWM  11

#define PIN_RIGHT_DIR 13 
#define PIN_RIGHT_PWM 12

#define PIN_ODOM_WHEEL_A 3
#define PIN_ODOM_WHEEL_B 2

#define PIN_ODOM_LEFT_A  21
#define PIN_ODOM_LEFT_B  20

#define PIN_ODOM_RIGHT_A 19
#define PIN_ODOM_RIGHT_B 18
