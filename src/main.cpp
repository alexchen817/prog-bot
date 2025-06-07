#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"

// object declarations
pros::Controller Controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_mg({3, 2}, pros::MotorGearset::green);
pros::MotorGroup right_mg({-1, -4}, pros::MotorGearset::green);
lemlib::Drivetrain drivetrain(&left_mg,                   // left motor group
                              &right_mg,                  // right motor group
                              15,                         // 15 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              200, // drivetrain rpm is 360
                              2    // horizontal drift is 2 (for now)
);
pros::Imu imu(20);
pros::adi::Encoder vertical_encoder('A', 'B');
pros::adi::Encoder horizontal_encoder('C', 'D');
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder,
                                                lemlib::Omniwheel::OLD_275,
                                                0.1);
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder,
                                              lemlib::Omniwheel::OLD_275, -2.5);

lemlib::OdomSensors sensors(
    &vertical_tracking_wheel, // vertical tracking wheel 1, set to null
    nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
    &horizontal_tracking_wheel, // horizontal tracking wheel 1
    nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a
             // second one
    &imu     // inertial sensor
);
// controls linear movement
lemlib::ControllerSettings
    lateral_controller(16, // proportional gain (kP)
                       0,  // integral gain (kI)
                       1,  // derivative gain (kD)
                       0,  // anti windup
                       0,  // small error range, in inches
                       0,  // small error range timeout, in milliseconds
                       0,  // large error range, in inches
                       0,  // large error range timeout, in milliseconds
                       0   // maximum acceleration (slew)
    );

// angular PID controller
lemlib::ControllerSettings
    angular_controller(2.15, // proportional gain (kP)
                       0,    // integral gain (kI)
                       0.95, // derivative gain (kD)
                       3,    // anti windup
                       3,    // small error range, in degrees
                       100,  // small error range timeout, in milliseconds
                       0.1,  // large error range, in degrees
                       500,  // large error range timeout, in milliseconds
                       0     // maximum acceleration (slew)
    );
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);
// lateral PID controller

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors
  // print position to brain screen
  pros::Task screen_task([&]() {
    while (true) {
      // print robot location to the brain screen
      pros::lcd::print(0, "X: %f", chassis.getPose().x);         // x
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);         // y
      pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
      // delay to save resources
      pros::delay(20);
    }
  });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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
  // Autonomous code goes here
  // set position to x:0, y:0, heading:0
  // turn to face heading 90 with a very long timeout
  //   chassis.turnToHeading(90, 5000);
  //   chassis.turnToHeading(-180, 5000);
  //   chassis.turnToHeading(270, 5000);
  //   chassis.turnToHeading(0, 5000);

  chassis.setPose(12, 12, 0);
  chassis.swingToHeading(35, DriveSide::RIGHT, 500);
  chassis.moveToPoint(71, 72, 2000);
  chassis.moveToPoint(13, 122, 3000);
  chassis.moveToPoint(76, 136, 3000);
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

  while (true) {
    // Arcade control scheme
    int dir = Controller.get_analog(
        ANALOG_LEFT_Y); // Gets amount forward/backward from left joystick
    int turn = Controller.get_analog(
        ANALOG_RIGHT_X);       // Gets the turn left/right from right joystick
    left_mg.move(dir + turn);  // Sets left motor voltage
    right_mg.move(dir - turn); // Sets right motor voltage
    pros::delay(20);           // Run for 20 ms then update
  }
}