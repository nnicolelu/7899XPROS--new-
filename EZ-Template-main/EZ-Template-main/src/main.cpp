#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-20, -19, 12},     // Left Chassis Ports (negative port will reverse it!)
    {15, 16, -17},  // Right Chassis Ports (negative port will reverse it!)

    4,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"SKILLS!", skills},
      {"Testing distance sensor", distanceTest},
      {"Left four ball autonomous\n\nLeft side autonomous that gets four balls into the long goal with descore arm hold", left4Ball},
      {"Right four ball autonomous\n\nRight side auto that gets four balls into long goal with descore", right4Ball},
      {"Left hold\n\nLeft side autonomous with descore arm hold", leftHold},
      {"Right hold\n\nRight side autonomous with descore arm hold", rightHold},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
  pros::lcd::initialize();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
                 // Reset gyro position to 0
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}

/**
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  static uint32_t lastPrint = 0;
  while (true) {
    uint32_t now = pros::millis();
    if (now - lastPrint >= 200) {   // 200 ms
      lastPrint = now;
      double frontInch = front.get() / 25.4; 
      double sideInch = side.get() / 25.4;
      pros::lcd::print(0, "Side: %f in, Front: %f in", sideInch, frontInch);
    }
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();
    chassis.opcontrol_arcade_standard(ez::SPLIT);
    // pneumatics
    descore.button_toggle(master.get_digital(DIGITAL_B));
    matchLoader.button_toggle(master.get_digital(DIGITAL_DOWN));
    stopPiston.button_toggle(master.get_digital(DIGITAL_Y));
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      stopPiston.set(false);
      bottomRollers.move(127);
      topRollers.move(127);
      topIntake.move(127);
      chassis.pid_drive_set(40_in, 50, true);
      pros::delay(1000);
      chassis.pid_drive_set(-10_in, 80, true);
      pros::delay(500);
      chassis.pid_drive_set(25_in, 50, true);
      pros::delay(800);
      chassis.pid_drive_set(-10_in, 80, true);
      pros::delay(500);
      chassis.pid_drive_set(23_in, 50, true);
      pros::delay(800);
      chassis.pid_drive_set(-10_in, 80, true);
      pros::delay(500);
      chassis.pid_drive_set(20_in, 50, true);
      pros::delay(1000);
      chassis.pid_drive_set(-24_in, 50, true);
      pros::delay(1600);
      chassis.pid_drive_set(10_in, 50, true); // finished with clearing park
      pros::delay(480);
      bottomRollers.move(0);
      topRollers.move(0);
      topIntake.move(0); 
      chassis.pid_drive_set(-32, 120, true);
      pros::delay(800);
      chassis.pid_turn_set(90, 90);
      pros::delay(500);
      bottomRollers.move(127);
      chassis.pid_drive_set(22, 120, true);
      pros::delay(700);
      chassis.pid_turn_set(43, 90);
      pros::delay(380);
      chassis.pid_drive_set(-16, 70, true); // middle goal
    }
    // intake logic
    if (master.get_digital(DIGITAL_R1)) { //scoring on the top
      topRollers.move(127);
      topIntake.move(127);
      if (master.get_digital(DIGITAL_L1)) { // picking balls from floor
        bottomRollers.move(127);
      }
      else {
        bottomRollers.move(0);
      }
    }
    else if (master.get_digital(DIGITAL_R2)) {
      topRollers.move(40);
      topIntake.move(-35);
    }
    else if (master.get_digital(DIGITAL_L1)) { // picking up from floor
      bottomRollers.move(127);
      if (master.get_digital(DIGITAL_R1)) { // scoring on top goal
        topRollers.move(127);
        topIntake.move(127);
      }
      else if (master.get_digital(DIGITAL_R2)) { // scoring on middle top 
        topRollers.move(40);
        topIntake.move(-35);
      }
      else {
        topRollers.move(0);
        topIntake.move(0);
      }
    }
    else if (master.get_digital(DIGITAL_L2)) { // removing balls from bot
      bottomRollers.move(-127);
      topRollers.move(-127);
      topIntake.move(-127);
    }
    else {
      bottomRollers.move(0);
      topRollers.move(0);
      topIntake.move(0);
    }
    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}