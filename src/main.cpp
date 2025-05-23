#include "main.h"
#include "motoritf.h"
/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////
bool alliance = true; //true = red false = blue
bool color_sort_enabled = false;
constexpr double BLUE_RING_HUE = 215;
constexpr double RED_RING_HUE = 5;
constexpr double RING_HUE_TOLOERANCE = 15;
inline chisel::Command driver_intake_command = {0, 326};
inline chisel::Command auton_intake_command = {0, 551};
inline chisel::Command unstuck_intake_command = {0, 0};
inline chisel::Command color_sort_command = {0, 0};
pros::Controller controller(pros::E_CONTROLLER_MASTER);
// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-9, -19, 7},     // Left Chassis Ports (negative port will reverse it!)
    {8, 20, -10},  // Right Chassis Ports (negative port will reverse it!)
    3,      // IMU Port
    2.75,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

inline auto intake_itf = chisel::MotorItf(&intake);
bool red_ring_seen = false;
bool blue_ring_seen = false;

uint32_t last_outtake = 0;

void setAllianceBlue() {
  alliance = false;
  color_sort_enabled = true;
}

void setAllianceRed() {
  alliance = true;
  color_sort_enabled = true;
}

void queue_outtake() {

  if (color_sort_enabled) {
    pros::delay(0);

    color_sort_command.power = -30;
    color_sort_command.priority = 999;

    pros::delay(200);

    color_sort_command.priority = 0;
  }
}

void color_sort_update() {
    const bool p_brs = blue_ring_seen;
    const bool p_rrs = red_ring_seen;

    blue_ring_seen = false;
    red_ring_seen = false;

    // if (!color_sort_enabled) return;

    const double r_hue_min = std::fmod(RED_RING_HUE - RING_HUE_TOLOERANCE + 360, 360.0f);
    const double r_hue_max = std::fmod(RED_RING_HUE + RING_HUE_TOLOERANCE, 360.0f);
    const double b_hue_min = std::fmod(BLUE_RING_HUE - RING_HUE_TOLOERANCE + 360, 360.0f);
    const double b_hue_max = std::fmod(BLUE_RING_HUE + RING_HUE_TOLOERANCE, 360.0f);

    bool b_ring = false;
    bool r_ring = false;

    if (optical.get_proximity() >= 200) {
        if (b_hue_min <= b_hue_max) {
            // if it's normal, just check the ranges
            b_ring = b_hue_min <= optical.get_hue() && optical.get_hue() <= b_hue_max;
        } else if (b_hue_min >= b_hue_max) {
            // if the range goes around 0, say hue min is 330 while hue_max is 30,
            // 0 and 360 are used as bounds.
            b_ring = optical.get_hue() <= b_hue_max || optical.get_hue() >= b_hue_min;
        }
        if (b_ring) blue_ring_seen = true;

        if (r_hue_min <= r_hue_max) {
            // if it's normal, just check the ranges
            r_ring = r_hue_min <= optical.get_hue() && optical.get_hue() <= r_hue_max;
        } else if (r_hue_min >= r_hue_max) {
            // if the range goes around 0, say hue min is 330 while hue_max is 30,
            // 0 and 360 are used as bounds.
            r_ring = optical.get_hue() <= r_hue_max || optical.get_hue() >= r_hue_min;
        }
        if (r_ring) red_ring_seen = true;
    }

    if (pros::millis() - last_outtake >= 500) {
        if ((alliance && !b_ring && p_brs || !alliance && !r_ring && p_rrs)) {
            pros::Task([] {
                queue_outtake();
                last_outtake = pros::millis();
            });
        }
    }
}


const int numStates = 2;
int states[numStates] = {3350, 17000};
int currState = 1;
int armTarget = 0;
int multiplier = 350;
bool armPIDEnabled = false;
double lastError = 0;
double totalError = 0;
bool isArmLocked = false;

int numIterationsWithinTolerance = 0;

void nextState(){
  currState = (currState + 1) % numStates;
  setArmTarget(states[currState]);
}

int convertArmPosition(int position) {
  return (position + 6000) % 36000 - 6000;
}

void liftArm(){
  double kp = 0.016;
  double kd = 0.07;
  double ki = 0.0001; 
  double offset = 18.0; // this is the minimum abs(velocity) even when error is zero. this is needed to overcome gravity of the arm.
  double tolerance = 80; // the tolerance for the arm to be considered "locked"
  int error_threshold_for_ki = 500; // the error threshold for ki to be added to the total error
  int convertedArmRotation = convertArmPosition(armRotation.get_position());
  double error = armTarget - convertedArmRotation;

  double changeInError = error - lastError;
  
  double kdTerm = (lastError == 0) ? 0 : changeInError * kd;
  double kiTerm = (abs(error) < error_threshold_for_ki) ? ki * totalError : 0; // only add to the total error if we are within 500 degrees of the target
  double velocity = kp * error + kiTerm + kdTerm; // PID equation
  
  // add the effect of gravity 
  if (convertedArmRotation < 9500 && error > 0) {
    velocity = velocity + offset;
  } else if (convertedArmRotation > 9500 && error < 0) {
    velocity = velocity - offset; 
  }

  if ((abs(error) < tolerance && !isArmLocked && numIterationsWithinTolerance > 10) || // if it's moving, only hold it if it's within 80 centidegrees of the target
      (abs(error) < 300 && isArmLocked)) { // if it's not moving, have a higher tolerance of movements before unlocking
    velocity = 0; // lock the arm motor
    totalError = 0; // reset the total error for ki
    isArmLocked = true;
  } else {
    isArmLocked = false; 
    if (abs(error) < error_threshold_for_ki) {
      totalError = totalError + error; // only add to the total error if we are within 500 degrees of the target
    } 

    if (abs(error) < tolerance) {
      numIterationsWithinTolerance++;
    } else {
      numIterationsWithinTolerance = 0; // reset the number of iterations within tolerance
    }
  } 
  
  

  lastError = error;

  // ez::screen_print("Arm Rotation: " + util::to_string_with_precision(convertedArmRotation)
  //   + ", " + util::to_string_with_precision(velocity) + 
  //   ", " + util::to_string_with_precision(error) + 
  //   ", " + util::to_string_with_precision(armTarget) + 
  //   ", " + util::to_string_with_precision(totalError), 6);

  if (velocity != 0) { arm.move(velocity); }
  else { arm.move_velocity(0); } // stop the motor if velocity is 0
  
}

void setArmTarget(int targetInCentidegrees) {
  armPIDEnabled = true;
  totalError = 0; // reest the total error for ki
  armTarget = targetInCentidegrees;
}

/*Old code
int lift_Arm() {
  ArmRotation.resetPosition();
  Arm.setStopping(hold);
  while (true) {
    double kp = 0.6;
    double error = target - ArmRotation.position(degrees);
    double velocity = kp * error;
    Arm.setVelocity(velocity, percent);
    Arm.spin(forward);
    wait(20, msec);
  }
  return 0;
}*/

bool determineColor(int redThreshold, int blueThreshold, char color, pros::c::optical_rgb_s_t rgb_value){
  if (rgb_value.red > redThreshold && rgb_value.blue < blueThreshold) {
    return true;
  }
  else if(rgb_value.red < redThreshold && rgb_value.blue > blueThreshold){
    return false;
  }
  else if(color == 'r'){
    return true;
  }
  else{
    return false;
  }
}

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
// ez::tracking_wheel horiz_tracker(8, 2.75, 4.0);  // This tracking wheel is perpendicular to the drive wheels
// ez::tracking_wheel vert_tracker(9, 2.75, 4.0);   // This tracking wheel is parallel to the drive wheels

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  intake_itf.assign_command(&color_sort_command);
  optical.set_integration_time(10);
  optical.set_led_pwm(100);
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(2.1, 4.3);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"Red Right", red_right_auton},
      {"Blue Right", blue_right_auton},
      {"Red Left", red_left_auton},
      {"Blue Left", blue_left_auton},
      {"Test Robot", test_robot},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
  });

  // Initialize chassis and auton selector
  armRotation.set_position(0);
  chassis.initialize();
  arm.set_brake_mode(pros::MotorBrake::hold);
  //armRotation.set_position(0);
  //armRotation.setPosition(0, degrees);
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");

  pros::Task liftControlTask([]{
    while(true){
      if(armPIDEnabled) {
        liftArm();
      }
      intake_itf.clean_commands();
      intake_itf.update();
      intake_itf.push_control();
      color_sort_update();
      pros::delay(10);
    }
  });

  // pros::Task colorSortTask([]{
  //   while(true){
  //     pros::c::optical_rgb_s_t rgb_value = opticalSensor.get_rgb();
  //     determineColor(100, 100, 'r', rgb_value);
  //     pros::delay(10);
  //   }
  // });
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
  // . . .
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
  intake_itf.assign_command(&auton_intake_command);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency
  //chassis.pid_tuner_toggle();
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

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
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
    ez::screen_print("Rotation: " + util::to_string_with_precision(chassis.drive_imu_get()), 7);
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
    if (master.get_digital(DIGITAL_A) && master.get_digital(DIGITAL_LEFT)) {
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
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  intake_itf.assign_command(&driver_intake_command);
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);

  // Trigger the selected autonomous routine
  // if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
  //   autonomous();
  // }

  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    if(master.get_digital_new_press(DIGITAL_UP)){
      color_sort_enabled = !color_sort_enabled;
      master.rumble(color_sort_enabled ? "." : "---");
    }

    // chassis.opcontrol_tank();  // Tank control
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    //intake

      if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
        driver_intake_command.power = 127;
      } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        driver_intake_command.power = -127;
      } else {
        driver_intake_command.power = 0;
      }
    
    // pros::c::optical_rgb_s_t rgb_value = colorSorter.get_rgb();
    // if(!determineColor(100, 100, 'r', rgb_value)){
    //   pros::delay(15);
    //   intake.move_velocity(-550*(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1) - controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)));
    // }
    
    //arm 
    if (!armPIDEnabled) {
      arm.move_velocity(0);
    }

    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      armPIDEnabled = false;
      if(convertArmPosition(armRotation.get_position()) <= 500 && master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        multiplier = 0;
      }
      else if(convertArmPosition(armRotation.get_position()) >= 25000 && master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        multiplier = 0;
      }
      else{
        multiplier = 100;
      }
      arm.move_velocity(multiplier*(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) - controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)));
    }
    
    
    if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)){
      nextState();
    }

    
 
    //mogo
    static bool mogoToggle { false };    //This static variable will keep state between loops or function calls
    if(controller.get_digital_new_press(DIGITAL_Y)) {
      mogo.set_value(!mogoToggle);    //When false go to true and in reverse
      mogoToggle = !mogoToggle;    //Flip the toggle to match piston state
    }

    //rightDoinker
    static bool rightDnkrToggle { false };    //This static variable will keep state between loops or function calls
    if(controller.get_digital_new_press(DIGITAL_B)) {
      rightDoinker.set_value(!rightDnkrToggle);    //When false go to true and in reverse
      rightDnkrToggle = !rightDnkrToggle;    //Flip the toggle to match piston state
    }

    //leftDoinker
    static bool leftDnkrToggle { false };    //This static variable will keep state between loops or function calls
    if(controller.get_digital_new_press(DIGITAL_DOWN)) {
      leftDoinker.set_value(!leftDnkrToggle);    //When false go to true and in reverse
      leftDnkrToggle = !leftDnkrToggle;    //Flip the toggle to match piston state
    }

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}

