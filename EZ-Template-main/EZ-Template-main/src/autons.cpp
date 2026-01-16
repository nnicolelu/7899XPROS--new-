#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 120;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(3.0, 0.0, 3.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(1.0, 0.0, 0.5);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.0, 20.0, 0.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

// . . .
// Make your own autonomous functions here!
// . . .
void simpleMoveFront(int targetInch) {
  double frontInch = front.get() / 25.4; 
  chassis.pid_drive_set((frontInch - targetInch), 100, true);
}

void driveOff() {
  chassis.pid_drive_set(6_in, DRIVE_SPEED, true);
  pros::delay(600);
  chassis.pid_wait();
}

void rightHold() {
  stopPiston.set(false);
  bottomRollers.move(127);
  topRollers.move(127);
  topIntake.move(127);
  chassis.pid_drive_set(34.5_in, DRIVE_SPEED, true);
  pros::delay(800);
  chassis.pid_turn_set(38_deg, TURN_SPEED);
  pros::delay(300);
  chassis.pid_drive_set(13_in, 110, true);
  pros::delay(300);
  matchLoader.set(true);
  chassis.pid_drive_set(9_in, 1110, true);
  pros::delay(500);
  matchLoader.set(false);
  chassis.pid_turn_set(130_deg, TURN_SPEED);
  pros::delay(450);
  chassis.pid_drive_set(41.5_in, DRIVE_SPEED, true); // going to goal
  pros::delay(1000);
  chassis.pid_turn_set(180_deg, DRIVE_SPEED);
  matchLoader.set(true);
  pros::delay(500);
  chassis.pid_drive_set(23_in, 80, true); // match loading
  pros::delay(1100);
  bottomRollers.move(0);
  chassis.pid_drive_set(-42_in, 90, true); // scoring
  pros::delay(800);
  stopPiston.set(true);
  matchLoader.set(false);
  bottomRollers.move(127);
  topRollers.move(127);
  topIntake.move(127);
  pros::delay(2000);
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  pros::delay(500);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  pros::delay(500);
  simpleMoveFront(6);
  pros::delay(650);
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  pros::delay(500);
  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  chassis.pid_wait();
}

void leftHold() {
  stopPiston.set(false);
  bottomRollers.move(127);
  topRollers.move(127);
  topIntake.move(127);
  chassis.pid_drive_set(34.5_in, DRIVE_SPEED, true);
  pros::delay(800);
  chassis.pid_turn_set(-40_deg, TURN_SPEED);
  pros::delay(300);
  chassis.pid_drive_set(15_in, 110, true);
  pros::delay(300);
  matchLoader.set(true);
  chassis.pid_drive_set(7_in, 1110, true);
  pros::delay(300);
  chassis.pid_turn_set(-130_deg, TURN_SPEED);
  pros::delay(450);
  chassis.pid_drive_set(39.5_in, DRIVE_SPEED, true); // going to goal
  pros::delay(900);
  chassis.pid_turn_set(-180_deg, DRIVE_SPEED);
  pros::delay(400);
  chassis.pid_drive_set(20_in, 80, true); // match loading
  pros::delay(1350);
  bottomRollers.move(0);
  chassis.pid_drive_set(-40_in, 90, true);
  pros::delay(800); // scoring
  stopPiston.set(true);
  bottomRollers.move(127);
  topRollers.move(127);
  topIntake.move(127);
  pros::delay(2000);
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  stopPiston.set(false);
  matchLoader.set(false);
  pros::delay(500);
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  pros::delay(500);
  chassis.pid_drive_set(16_in, DRIVE_SPEED, true);
  pros::delay(600);
  chassis.pid_turn_set(-180_deg, TURN_SPEED);
  pros::delay(500);
  chassis.pid_drive_set(-28.5_in, DRIVE_SPEED, true);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  pros::delay(5000);
  chassis.pid_wait();
}

void left4Ball() {
  stopPiston.set(false);
  chassis.pid_drive_set(44_in, DRIVE_SPEED, true); 
  pros::delay(900);
  chassis.pid_turn_set(-90_deg, TURN_SPEED); 
  matchLoader.set(true);
  pros::delay(500);
  bottomRollers.move(127);
  topRollers.move(127);
  topIntake.move(127);
  chassis.pid_drive_set(18_in, 100, true);
  pros::delay(1000); // match loading
  chassis.pid_drive_set(-37_in, 100, true);
  pros::delay(1000);
  matchLoader.set(false);
  stopPiston.set(true);
  pros::delay(800);
  bottomRollers.move(0);
  topRollers.move(0);
  topIntake.move(0);
  chassis.pid_drive_set(14_in, DRIVE_SPEED, true);
  pros::delay(300);
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  pros::delay(350);
  chassis.pid_drive_set(19_in, DRIVE_SPEED, true);
  pros::delay(450);
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  pros::delay(450);
  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true);
  pros::delay(500);
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
  chassis.pid_wait();
}

void right4Ball() {

}

void skills() {
  stopPiston.set(false);
  bottomRollers.move(127);
  topRollers.move(127);
  topIntake.move(127);
  chassis.pid_drive_set(40_in, 60, true);
  pros::delay(1000);
  chassis.pid_drive_set(-11_in, 70, true);
  pros::delay(500);
  chassis.pid_drive_set(20_in, 60, true);
  pros::delay(800);
  chassis.pid_drive_set(-11_in, 70, true);
  pros::delay(500);
  chassis.pid_drive_set(20_in, 60, true);
  pros::delay(800);
  chassis.pid_drive_set(-11_in, 70, true);
  pros::delay(500);
  chassis.pid_drive_set(20_in, 60, true);
  pros::delay(800);
  chassis.pid_drive_set(-11_in, 70, true);
  pros::delay(500);
  topRollers.move(0);
  chassis.pid_drive_set(23_in, 60, true);
  pros::delay(1000);
  chassis.pid_drive_set(-24_in, 60, true);
  pros::delay(1400);
  bottomRollers.move(0);
  topIntake.move(0);
  chassis.pid_drive_set(5_in, 70, true); // finished with clearing park
  simpleMoveFront(14);
  pros::delay(480);
  chassis.pid_drive_set(-32_in, DRIVE_SPEED, true);
  pros::delay(800);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  pros::delay(500);
  bottomRollers.move(127);
  chassis.pid_drive_set(21_in, DRIVE_SPEED, true); // red block
  pros::delay(700);
  chassis.pid_turn_set(43_deg, TURN_SPEED);
  pros::delay(380);
  chassis.pid_drive_set(-16_in, 70, true); // middle goal
  pros::delay(800);
  bottomRollers.move(-100);
  topRollers.move(-127);
  topIntake.move(-127);
  pros::delay(400);
  bottomRollers.move(90);
  topRollers.move(40);
  topIntake.move(-55);
  pros::delay(2000);
  chassis.pid_drive_set(6_in, DRIVE_SPEED, true);
  bottomRollers.move(90);
  topRollers.move(40);
  topIntake.move(-40);
  pros::delay(500);
  chassis.pid_drive_set(-6_in, DRIVE_SPEED, true);
  pros::delay(300);
  chassis.pid_turn_set(43_deg, TURN_SPEED);
  pros::delay(380);
  chassis.pid_drive_set(50_in, DRIVE_SPEED, true);
  pros::delay(1000);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  pros::delay(350);
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  simpleMoveFront(18); // checking distance for match laoder
  pros::delay(600);
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  pros::delay(450);
  matchLoader.set(true);
  bottomRollers.move(127);
  topRollers.move(127);
  topIntake.move(127);
  chassis.pid_drive_set(22_in, 80, true); // getting first match loader
  pros::delay(2100);
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true);
  pros::delay(500);
  matchLoader.set(false);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  pros::delay(300);
  bottomRollers.move(0);
  topRollers.move(0);
  topIntake.move(0);
  chassis.pid_drive_set(3_in, DRIVE_SPEED, true);
  pros::delay(200);
  simpleMoveFront(4); // checking distance before turning to 2nd quadrant
  pros::delay(500);
  chassis.pid_turn_set(-180_deg, TURN_SPEED);
  pros::delay(500);
  chassis.pid_drive_set(70_in, DRIVE_SPEED, true);
  pros::delay(1100);
  simpleMoveFront(30);
  pros::delay(600);
  chassis.pid_turn_set(250_deg, TURN_SPEED);
  pros::delay(500);
  chassis.pid_drive_set(27_in, DRIVE_SPEED, true);
  pros::delay(300);
  chassis.pid_turn_set(-180_deg, TURN_SPEED);
  pros::delay(400);
  chassis.pid_drive_set(-22_in, 100, true); // scoring first match loader
  pros::delay(600);
  chassis.pid_turn_set(188_deg, TURN_SPEED);
  pros::delay(500);
  stopPiston.set(true);
  topIntake.move(127);
  bottomRollers.move(127);
  topRollers.move(127);
  pros::delay(1800);
  stopPiston.set(false);
  matchLoader.set(true);
  chassis.pid_drive_set(36_in, 80, true); // second match loader
  pros::delay(2500);
  chassis.pid_drive_set(-37_in, 70, true); // scoring second match loader
  pros::delay(800);
  stopPiston.set(true);
  pros::delay(1600);
  matchLoader.set(false);
  chassis.pid_drive_set(15_in, 100, true);
  pros::delay(400);
  stopPiston.set(false);
  chassis.pid_drive_set(-16_in, 80, true); // push for control
  pros::delay(400);
  chassis.pid_drive_set(18_in, DRIVE_SPEED, true);
  pros::delay(600);
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  pros::delay(400);
  //replace with code for blue park zone
  chassis.pid_drive_set(80_in, 100, true); // driving to 3rd goal (switch to use distance sensor)
  pros::delay(1300);
  simpleMoveFront(19);
  pros::delay(700);
  // replace with code for blue park zone
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  pros::delay(600);
  chassis.pid_drive_set(-24_in, 80, true);
  pros::delay(800);
  matchLoader.set(true);
  chassis.pid_drive_set(35_in, 50, true); // getting 3rd match loader
  pros::delay(1000);
  stopPiston.set(false);
  bottomRollers.move(127);
  topRollers.move(127);
  topIntake.move(127);
  pros::delay(2000); 
  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true);
  pros::delay(500);
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  topRollers.move(0);
  topIntake.move(0);  
  pros::delay(400);
  matchLoader.set(false);
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  pros::delay(350);
  bottomRollers.move(0);
  simpleMoveFront(4.5);
  pros::delay(500);
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  pros::delay(600);  // 450
  chassis.pid_drive_set(60_in, 90, true); // going to 4th quadrant
  pros::delay(1100);
  simpleMoveFront(25);
  pros::delay(700);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  pros::delay(500);
  chassis.pid_drive_set(18_in, DRIVE_SPEED, true);
  pros::delay(500);
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  pros::delay(500);
  chassis.pid_drive_set(-18_in, 80, true); // scoring 3rd match loader
  pros::delay(500);
  stopPiston.set(true);
  topRollers.move(127);
  topIntake.move(127);
  bottomRollers.move(127);
  pros::delay(1800);
  stopPiston.set(false);
  matchLoader.set(true);
  chassis.pid_drive_set(36_in, 50, true); // getting 4th match loader
  pros::delay(3000);
  chassis.pid_drive_set(-36_in, 80, true); // scoring 4th match loader
  pros::delay(800);
  stopPiston.set(true);
  pros::delay(1800);
  matchLoader.set(false);
  chassis.pid_drive_set(15_in, 100, true);
  pros::delay(400);
  stopPiston.set(false);
  chassis.pid_drive_set(-16_in, 100, true); // push for control
  pros::delay(400);
  chassis.pid_drive_set(18_in, DRIVE_SPEED, true); // going to park
  pros::delay(500);
  chassis.pid_turn_set(45_deg, TURN_SPEED);
  pros::delay(300);
  chassis.pid_drive_set(33_in, DRIVE_SPEED, true);
  pros::delay(700);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  pros::delay(350);
  bottomRollers.move(127);
  topRollers.move(127);
  chassis.pid_drive_set(45_in, DRIVE_SPEED, true); // parked
  chassis.pid_wait();
}

void soloAWP()  {
  stopPiston.set(false);
  matchLoader.set(false);
  bottomRollers.move(127);
  topRollers.move(127);
  topIntake.move(127);
   //4 ball long goal right
  chassis.pid_drive_set(31_in, DRIVE_SPEED, true);
  pros::delay(1000);
  chassis.pid_turn_set(89_deg, TURN_SPEED);
  pros::delay(200);
  matchLoader.set(true);
  pros::delay(200); 
  chassis.pid_drive_set(13_in, DRIVE_SPEED, true); //12_in
  pros::delay(900);
  chassis.pid_drive_set(-35_in, DRIVE_SPEED, true);
  pros::delay(700);
  stopPiston.set(true);
  pros::delay(600);
  matchLoader.set(false);
  pros::delay(300);
  //6 ball pickup + middle goal
  chassis.pid_turn_set(220_deg, TURN_SPEED);
  pros::delay(900);
  stopPiston.set(false);
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true);
  pros::delay(600);
  matchLoader.set(true);
  pros::delay(300);
  chassis.pid_turn_set(179_deg, TURN_SPEED);
  pros::delay(500);
  matchLoader.set(false);
  chassis.slew_drive_set(false);  // Disables global slew
  chassis.slew_drive_constants_set(-10_in, 40);
  chassis.pid_drive_set(50_in, 80, true); //48_in=
  pros::delay(1000); //1000
  matchLoader.set(true);
  pros::delay(200); //test
  //chassis.pid_drive_set(-7_in, DRIVE_SPEED, true); //test
  //pros::delay(300);
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  matchLoader.set(false);
  pros::delay(500);
  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true);
  pros::delay(800);
  topRollers.move(-127);
  bottomRollers.move(-127);
  topIntake.move(-127);
  pros::delay(300);
  topIntake.move(-50);
  topRollers.move(60);
  bottomRollers.move(90);
  pros::delay(800);
  topRollers.move(0);
  pros::delay(100);
  topIntake.move(127);
  topRollers.move(127);
  //other long goal
  chassis.pid_drive_set(54_in, DRIVE_SPEED, true);
  pros::delay(1300);
  chassis.pid_turn_set(89_deg, TURN_SPEED);
  pros::delay(500);
  chassis.pid_drive_set(19_in, DRIVE_SPEED, true);
  pros::delay(1100);
  chassis.pid_drive_set(-35_in, DRIVE_SPEED, true);
  pros::delay(800);
  stopPiston.set(true);
  pros::delay(3000);
  chassis.pid_wait();
}