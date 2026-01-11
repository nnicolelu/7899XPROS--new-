#include "EZ-Template/drive/drive.hpp"
#include <cmath>

/**
 * @brief Clamp a value within a given range.
 * 
 * @param v The value to be clamped.
 * @param lo The lower bound of the range.
 * @param hi The upper bound of the range.
 * 
 * @return The clamped value, which is guaranteed to be within the range [lo, hi].
 */
static inline double clampd(double v, double lo, double hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/**
 * @brief Converts millimeters to inches
 * 
 * @param mm The distance in millimeters
 * 
 * @return The distance in inches
 */
static inline double mm_to_in(double mm) {
  return mm / 25.4;
}

/**
 * @brief A simple low-pass filter implementation.
 * 
 * This function applies a low-pass filter to a given value, to reduce noise and
 * oscillations. The algorithm is as follows: the current value is combined with the
 * previous filtered value, with a weighting factor of alpha. The result is the
 * filtered value.
 * 
 * @param prev The previous filtered value.
 * @param cur The current value to be filtered.
 * @param alpha The weighting factor, defaulting to 0.25.
 * 
 * @return The filtered value.
 */
static inline double lpf(double prev, double cur, double alpha = 0.25) {
  return prev + alpha * (cur - prev);
}

/**
 * Initializes the distance sensors for the drive subsystem.
 *
 * This function sets the front and side distance sensors for the drive subsystem.
 *
 * @param front The front distance sensor.
 * @param side The side distance sensor.
 */
void Drive::distance_sensor_init(pros::Distance* front, pros::Distance* side) {
  dist_front = front;
  dist_side = side;
}
/**
 * @brief Gets the filtered front distance sensor value.
 *
 * This function applies a low-pass filter to the front distance sensor value
 * to reduce noise and oscillations. The algorithm is as follows: the current
 * value is combined with the previous filtered value, with a weighting factor
 * of alpha. The result is the filtered value.
 *
 * @param prev The previous filtered value.
 *
 * @return The filtered front distance sensor value in inches.
 */

double Drive::distance_front_in_filtered(double prev) {
  if (!dist_front) return prev;

  double raw = mm_to_in(dist_front->get());
  if (raw < 0.5 || raw > 120.0) return prev;   // prevent 0/garbage values
  return lpf(prev, raw);
}


/**
 * @brief Drive to a target distance in front of the robot
 *
 * This function uses a slow P control to drive the robot to a target distance
 * in front of the robot. The function will timeout after a specified amount of time
 * if the target distance has not been reached.
 *
 * @param target_in The target distance in inches.
 * @param timeout_ms The timeout in milliseconds.
 */
void Drive::drive_to_front_distance(double target_in, int timeout_ms) { // HERE
  if (!dist_front) return;

  const double kp = 10.0;        // Slow P control
  const int maxV  = 45;          // maximum speed
  const double stopBand = 0.35;  // inch
  const int stableNeed = 1; // Nicole!!! May need to tune it to smaller value

  int stable = 0;
  double prev = mm_to_in(dist_front->get());
  uint32_t t0 = pros::millis();

  while (pros::millis() - t0 < timeout_ms) {

    double cur = distance_front_in_filtered(prev);
    prev = cur;

    double err = cur - target_in;    // Nicole!!! Note it may need to be reversed if the direction is reversed!!!
    double cmd = kp * err;

    if (cmd >  maxV) cmd =  maxV;
    if (cmd < -maxV) cmd = -maxV;

    if (std::fabs(err) < stopBand) stable++;
    else stable = 0;

    if (stable >= stableNeed) break;

    // Use existing raw motor interface to set the drive motors to the command value
    private_drive_set(cmd, cmd);

    pros::delay(20);
  }

  private_drive_set(0, 0);
} // drive_to_front_distance

// Drive::pros::Distance* dist_side = nullptr;

/**
 * @brief Applies a low-pass filter to the side distance sensor value
 * 
 * This function applies a low-pass filter to the side distance sensor value
 * to reduce noise and oscillations. The algorithm is as follows: the current
 * value is combined with the previous filtered value, with a weighting factor
 * of alpha. The result is the filtered value.
 * 
 * @param prev The previous filtered value.
 * 
 * @return The filtered side distance sensor value in inches.
 */
double Drive::distance_side_in_filtered(double prev) {
  if (!dist_side) return prev;
  double raw = mm_to_in(dist_side->get());
  if (raw < 0.5 || raw > 120.0) return prev;     // prevent 0 / garbage values
  return lpf(prev, raw, 0.25);
}

/**
 * Align the robot to the side wall.
 *
 * @param target_in The target distance to the side wall in inches.
 * @param timeout_ms The maximum time to wait for the robot to align to the side wall in milliseconds.
 *
 * This function aligns the robot to the side wall by using the side distance sensor and the IMU.
 * It generates commands for the left and right motors to align the robot to the side wall.
 * The function returns when the robot is within a certain distance to the side wall (stopBand) or when the time limit is reached (timeout_ms).
 * The function will not work if the side distance sensor is not set.
 */
void Drive::alignToSideWall(double target_in, int timeout_ms) {
  if (!dist_side) return;

  // ---- parameters (default) ----
  const double kWall = 9.0;      // side distance correct strength
  const double kHead = 1.6;      // heading correction strength
  const int    maxV  = 45;       // maximum speed (slow)
  const double stopBand = 0.35;  // inch
  const int    stableNeed = 6;

  int stable = 0;
  double prevSide = dist_side->get() / 25.4;
  double holdHeading = drive_imu_get();  // current angle, i.e., targer angle

  uint32_t t0 = pros::millis();

  while ((int)(pros::millis() - t0) < timeout_ms) {

    // --- distance to side wall (with filter + protection) ---
    double sideIn = distance_side_in_filtered(prevSide);
    prevSide = sideIn;

    double errWall = sideIn - target_in;   // Nicole!!! Note it may need to be reversed if the direction is reversed!!!

    // --- heading error ---
    double errHead = holdHeading - drive_imu_get(); // Nicole!!! Note it may need to be reversed if the direction is reversed!!!
    while (errHead > 180) errHead -= 360;
    while (errHead < -180) errHead += 360;

    if (std::fabs(errWall) < stopBand) stable++;
    else stable = 0;

    if (stable >= stableNeed) break;

    // --- generate left output and right output ---
    double steer = kWall * errWall + kHead * errHead;

    double left  =  steer;
    double right = -steer;

    if (left  >  maxV) left  =  maxV;
    if (left  < -maxV) left  = -maxV;
    if (right >  maxV) right =  maxV;
    if (right < -maxV) right = -maxV;

    private_drive_set((int)left, (int)right);
    pros::delay(20);
  }

  private_drive_set(0, 0);
} // alignToSideWall

/**
 * Drive the robot forward while following a side wall at a target distance.
 * The robot will stop when it reaches the target distance or the timeout is reached.
 * The robot will adjust its heading to maintain a constant distance from the side wall.
 * The robot will also slow down when it is close to the target distance to ensure a smooth stop.
 *
 * @param forward_in The target distance to travel in inches.
 * @param target_wall_in The target distance from the side wall in inches.
 * @param hold_heading_deg The target heading to maintain in degrees.
 * @param timeout_ms The maximum time to wait for the robot to reach the target distance in milliseconds.
 */
void Drive::drive_follow_sidewall(double forward_in,
                                  double target_wall_in,
                                  double hold_heading_deg,
                                  int timeout_ms) {
  if (!dist_side) return;

  // ----- tunable parameters（ default ） -----
  const double kWall = 7.0;        // wall distance adjustment strength (larger means stronger, but may lead to zigzag)
  const double kHead = 1.4;        // heading correction strength (larger means stronger, but may lead to jitter)
  const int    base  = 42;         // forward base speed (slower is more stable) // Nicole!!! Need tuning
  const int    maxV  = 65;         // maximum output limit
  const double stopBandIn = 0.6;   // stop when within this distance
  const int    stableNeed = 6;  // Nicole!!! Need to tune it to smaller value

  // start point (encoder inches)
  // Drive already have drive_sensor_left/right()
  const double start = (drive_sensor_left() + drive_sensor_right()) * 0.5;
  int stable = 0;
  double prevSide = mm_to_in(dist_side->get());
  uint32_t t0 = pros::millis();

  while ((int)(pros::millis() - t0) < timeout_ms) {

    // --- 1) current point（encoder inches） ---
    double cur = (drive_sensor_left() + drive_sensor_right()) * 0.5;
    double traveled = cur - start;
    double remain = forward_in - traveled;

    if (std::fabs(remain) < stopBandIn) stable++;
    else stable = 0;
    if (stable >= stableNeed) break;

    // --- 2) read distance to side wall (with filter + protection)
    double sideIn = distance_side_in_filtered(prevSide);
    prevSide = sideIn;

    // --- 3) distance to side wall error in inch ( >0 means further. Note it may need set be negative if the direction is reversed)
    double errWall = (sideIn - target_wall_in); // Nicole!!! Note it may need to be reversed if the direction is reversed!!!

    // --- 4) heading error (degrees) ---
    double h = drive_imu_get(); // current IMU reading
    double errHead = hold_heading_deg - h; //Nicole!!! Note it may need to be reversed if the direction is reversed!!!
    // wrap to [-180, 180]
    while (errHead > 180) errHead -= 360;
    while (errHead < -180) errHead += 360;

    // --- 5) heading adjustment ---
    // steer > 0 will let the robot turn right (left-right difference)
    // if steer
    double steer = kWall * errWall + kHead * errHead;

    // forward base speed: last few inches automatically slow down (more stable)
    double fwd = base;
    if (remain < 10.0) fwd = 34; // Nicole!!! Need tuning
    if (remain < 5.0)  fwd = 28; // Nicole!!! Need tuning

    // left + right output
    double left  = fwd + steer;
    double right = fwd - steer;

    left  = clampd(left,  -maxV, maxV);
    right = clampd(right, -maxV, maxV);

    private_drive_set((int)left, (int)right);
    pros::delay(20);
  }

  private_drive_set(0, 0);
} // drive_follow_sidewall
