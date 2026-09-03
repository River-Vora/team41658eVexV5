#include "main.h"
#include "lemlib/api.hpp"

/**
 * Enum representing the available drive styles.
 *
 * ARCADE: one joystick controls forward/back and turn together.
 * TANK: left and right joysticks control each side separately.
 */
enum class DriveMode {
	ARCADE,
	TANK
};

/**
 * Keeps track of the current drive mode so the robot knows whether it is in
 * arcade or tank control.
 */
static DriveMode current_drive_mode = DriveMode::ARCADE;

/**
 * The drivetrain is a pair of motor groups, one on the left and one on the
 * right. These are created once at file scope so they are easy to access from
 * helper functions and the main control loop.
 */
static pros::MotorGroup left_mg({1, 2});
static pros::MotorGroup right_mg({3, 4});

/**
 * LemLib Drivetrain configuration with the motor groups and chassis dimensions.
 *
 * 6-wheel drivetrain: 4 motors (1,2 left | 3,4 right) power all 6 wheels through gears.
 * Each side has 3 wheels mechanically linked (front omni, middle traction, back omni).
 * All wheels rotate at same RPM on each side.
 *
 * These values will be tuned once the actual robot is built:
 * - track_width: distance in inches between the left and right wheel centers
 * - wheel_diameter: 4 inches (all wheels are 4" diameter)
 * - rpm: motor RPM (360 for 36:1 gearset)
 */
static lemlib::Drivetrain drivetrain(
    &left_mg,
    &right_mg,
    11.5,   // track width in inches - TUNE FOR ACTUAL ROBOT
    4.0,    // wheel diameter in inches (all wheels 4")
    360,    // motor rpm (assumes 36:1 gearset) - VERIFY GEARSET
    8       // horizontalDrift: 8 for traction wheels in middle
);

/**
 * LemLib Controller Settings for autonomous linear movement (forward/backward).
 * These PID constants are used for path-following and will be tuned later.
 * Currently set conservatively for safety.
 */
static lemlib::ControllerSettings linearSettings(
    10,     // proportional gain (kP) - TUNE
    0,      // integral gain (kI) - TUNE
    0,      // derivative gain (kD) - TUNE
    3,      // integral anti windup range - TUNE
    10,     // small error range (inches) - TUNE
    100,    // small error timeout (ms) - TUNE
    3,      // large error range (inches) - TUNE
    500,    // large error timeout (ms) - TUNE
    5       // max acceleration (slew) - TUNE
);

/**
 * LemLib Controller Settings for autonomous angular movement (turning).
 * These PID constants are used for heading control and will be tuned later.
 * Currently set conservatively for safety.
 */
static lemlib::ControllerSettings angularSettings(
    2,      // proportional gain (kP) - TUNE
    0,      // integral gain (kI) - TUNE
    10,     // derivative gain (kD) - TUNE
    3,      // integral anti windup range - TUNE
    10,     // small error range (degrees) - TUNE
    100,    // small error timeout (ms) - TUNE
    3,      // large error range (degrees) - TUNE
    500,    // large error timeout (ms) - TUNE
    5       // max acceleration (slew) - TUNE
);

// ============================================
// SENSOR OBJECTS (Odometry & Game)
// ============================================

/**
 * Inertial Measurement Unit (IMU) for heading tracking.
 * Port 9. Provides gyroscope data for odometry rotation.
 */
static pros::Imu imu(9);

/**
 * GPS sensor for global position tracking (optional, polled separately).
 * Port 15. Can be used for field-based positioning.
 */
static pros::Gps gps(15);

/**
 * AI Vision sensor for object detection (optional, polled separately).
 * Port 20. Can be used for autonomous alignment and game object tracking.
 */
static pros::AIVision ai_vision(20);

/**
 * OdomSensors configuration for the Chassis.
 * Uses the IMU for heading. GPS is available separately for global position.
 */
static lemlib::OdomSensors sensors(
	nullptr,  // no vertical tracking wheels yet
	nullptr,  // no second vertical tracking wheel
	nullptr,  // no horizontal tracking wheels yet
	nullptr,  // no second horizontal tracking wheel
	&imu      // inertial measurement unit
);

/**
 * LemLib Exponential Drive Curves for smooth, responsive joystick control.
 *
 * The curves are applied to throttle (forward/backward) and steer (turn) inputs
 * separately, giving the driver fine control at low speeds and full power at high speeds.
 * These curves are also used by the Chassis for autonomous smoothing.
 */
static lemlib::ExpoDriveCurve throttleCurve(5, 0, 1.8);  // deadzone=5, minOutput=0, curveGain=1.8
static lemlib::ExpoDriveCurve steerCurve(5, 0, 1.8);

/**
 * LemLib Chassis object that unifies the drivetrain, sensors, and motion control.
 *
 * This object is the central hub for all robot motion. It handles:
 * - Driver control input processing via the curves
 * - Autonomous motion once odometry sensors are added
 * - Unified configuration for consistent behavior across all modes
 */
static lemlib::Chassis chassis(
    drivetrain,
    linearSettings,
    angularSettings,
    sensors,
    &throttleCurve,
    &steerCurve
);

// ============================================
// SUBSYSTEM MOTORS & STATE
// ============================================

/**
 * Cascade lift motor.
 * Port 7. Uses one motor for elevation.
 */
static pros::Motor lift_motor(7);

/**
 * Counter-roller intake motor.
 * Port 5. 11W motor.
 */
static pros::Motor counter_roller_motor(5);

/**
 * Top-roller intake motor.
 * Port 6. 11W motor.
 */
static pros::Motor top_roller_motor(6);

/**
 * Double-acting pneumatic claw.
 * The solenoid's electrical control wire uses ADI port 'A'.
 * Its physical A/B air ports handle extend and retract.
 */
static pros::adi::Pneumatics claw_open('A', false);

/**
 * Intake state: whether the intake is currently spinning.
 */
static bool intake_active = false;

/**
 * Claw state: 0 = open, 1 = closed.
 */
static int claw_state = 0;

// ============================================
// INTAKE SUBSYSTEM FUNCTIONS
// ============================================

/**
 * Spin both intake rollers inward to collect objects.
 */
void intake_spin_in() {
	counter_roller_motor.move(127);
	top_roller_motor.move(127);
	intake_active = true;
}

/**
 * Spin both intake rollers outward to eject objects.
 */
void intake_spin_out() {
	counter_roller_motor.move(-127);
	top_roller_motor.move(-127);
	intake_active = true;
}

/**
 * Stop the intake motor.
 */
void intake_stop() {
	counter_roller_motor.move(0);
	top_roller_motor.move(0);
	intake_active = false;
}

/**
 * Toggle intake between on (in) and off.
 */
void toggle_intake() {
	if (intake_active) {
		intake_stop();
	} else {
		intake_spin_in();
	}
}

// ============================================
// CASCADE LIFT SUBSYSTEM FUNCTIONS
// ============================================

/**
 * Raise the cascade lift at full power.
 */
void lift_raise() {
	lift_motor.move(127);
}

/**
 * Lower the cascade lift at full power.
 */
void lift_lower() {
	lift_motor.move(-127);
}

/**
 * Stop the cascade lift (coast).
 */
void lift_stop() {
	lift_motor.move(0);
}

/**
 * Hold the cascade lift at current position with gentle holding power.
 */
void lift_hold() {
	lift_motor.move(10);
}

// ============================================
// PNEUMATIC CLAW SUBSYSTEM FUNCTIONS
// ============================================

/**
 * Open the claw by activating the open solenoid.
 */
void claw_open_cmd() {
	claw_open.extend();
	claw_state = 0;
}

/**
 * Close the claw by activating the close solenoid.
 */
void claw_close_cmd() {
	claw_open.retract();
	claw_state = 1;
}

/**
 * Toggle the claw between open and closed.
 */
void toggle_claw() {
	if (claw_state == 0) {
		claw_close_cmd();
	} else {
		claw_open_cmd();
	}
}

/**
 * Updates the screen text on the V5 brain so the driver can see which drive mode
 * is active at all times.
 */
void update_drive_mode_ui() {
	switch (current_drive_mode) {
		case DriveMode::ARCADE:
			pros::lcd::set_text(0, "Drive: ARCADE");
			pros::lcd::set_text(1, "Use left stick");
			break;
		case DriveMode::TANK:
			pros::lcd::set_text(0, "Drive: TANK");
			pros::lcd::set_text(1, "Use both sticks");
			break;
	}
}

/**
 * Sets the current drive mode and immediately updates the brain display.
 *
 * @param mode The drive mode to apply.
 */
void set_drive_mode(DriveMode mode) {
	current_drive_mode = mode;
	update_drive_mode_ui();
}

/**
 * Toggles between arcade and tank drive.
 *
 * This is the function that gets called when the brain button is pressed.
 */
void toggle_drive_mode() {
	if (current_drive_mode == DriveMode::ARCADE) {
		set_drive_mode(DriveMode::TANK);
	} else {
		set_drive_mode(DriveMode::ARCADE);
	}
}

/**
 * Callback for the center button on the V5 brain.
 *
 * Pressing this button toggles the robot between arcade and tank drive.
 */
void on_center_button() {
	toggle_drive_mode();
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * This function initializes the brain screen, registers button callbacks, calibrates
 * odometry (IMU, tracking wheels), configures brake modes for all subsystems, and sets
 * the initial robot pose.
 */
void initialize() {
	// Initialize brain LCD
	pros::lcd::initialize();
	pros::lcd::clear();
	pros::lcd::set_text(0, "Team 41658E: Optimus Prime");
	pros::lcd::set_text(1, "Initializing...");
	pros::lcd::set_text(2, "Center = Toggle Drive");
	update_drive_mode_ui();

	// Set brake modes for drivetrain
	left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	// Set brake modes for subsystems
	counter_roller_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	top_roller_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	// Calibrate the IMU and start LemLib's odometry task.
	chassis.calibrate();

	// Set initial robot pose to (0, 0, 0)
	chassis.setPose(0, 0, 0);

	// Register brain button callback
	pros::lcd::register_btn1_cb(on_center_button);

	// Initial claw state: open
	claw_open_cmd();

	pros::lcd::clear();
	pros::lcd::set_text(0, "41658E Ready");
	pros::lcd::set_text(1, "Drive: ARCADE");
	pros::lcd::set_text(2, "Press Center to Toggle");
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
 * mode.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * This function handles:
 * - Drivetrain control through LemLib's arcade/tank methods and drive curves
 * - Intake control (X button toggles spin in/out)
 * - Lift control (UP/DOWN buttons, or press and hold)
 * - Claw control (A/B buttons, or L1 to toggle)
 * - Odometry updates for position tracking
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	// Lift state: for continuous raise/lower
	bool lift_raising = false;
	bool lift_lowering = false;

	while (true) {
		// ====================
		// DRIVETRAIN CONTROL
		// ====================

		if (current_drive_mode == DriveMode::ARCADE) {
			chassis.arcade(
				master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
				master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)
			);
		} else {
			chassis.tank(
				master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
				master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)
			);
		}

		// ====================
		// INTAKE CONTROL
		// ====================

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
			toggle_intake();
		}

		// ====================
		// CASCADE LIFT CONTROL
		// ====================

		if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
			lift_raise();
			lift_raising = true;
			lift_lowering = false;
		} else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
			lift_lower();
			lift_raising = false;
			lift_lowering = true;
		} else {
			// If neither button pressed and we were moving, hold position
			if (lift_raising || lift_lowering) {
				lift_hold();
			}
			lift_raising = false;
			lift_lowering = false;
		}

		// ====================
		// PNEUMATIC CLAW CONTROL
		// ====================

		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
			claw_open_cmd();
		}
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
			claw_close_cmd();
		}

		// Alternative: L1 toggle
		if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
			toggle_claw();
		}

		// ====================
		// ODOMETRY & LOOP DELAY
		// ====================

		// Chassis odometry is updated automatically by LemLib's background task.
		// No manual update call needed here.

		pros::delay(20);
	}
}