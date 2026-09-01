#include "main.h"

#include <cmath>

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
 * Drives should feel smooth and responsive, not twitchy. These constants control
 * our driver tuning before we start adding other subsystems.
 */
constexpr int JOYSTICK_DEADZONE = 5;
constexpr float DRIVE_CURVE_EXPONENT = 1.8f;
constexpr float MAX_SLEW_STEP = 22.0f;

/**
 * Tracks the previous command sent to each side of the drivetrain so the robot
 * can ramp its output in a controlled way instead of changing instantly.
 */
static float previous_left_output = 0.0f;
static float previous_right_output = 0.0f;

/**
 * Removes a small deadzone from the joystick input.
 *
 * The driver should not accidentally move the robot while the stick is barely
 * touched, but the deadzone should be small enough that fine control still feels
 * natural.
 *
 * @param input The raw joystick value from -127 to 127.
 * @return The input with the small deadzone removed.
 */
int apply_deadzone(int input) {
	if (std::abs(input) < JOYSTICK_DEADZONE) {
		return 0;
	}
	return input;
}

/**
 * Applies an exponential curve to joystick input so the robot feels more precise
 * around center and stronger as the stick moves farther away.
 *
 * This creates a better driver feel than a perfectly linear response because it
 * gives the driver more control at low speeds while still allowing full power
 * at the ends of the stick travel.
 *
 * @param input The joystick value after deadzone removal.
 * @return A curved output in the range -127 to 127.
 */
int apply_exponential_curve(int input) {
	if (input == 0) {
		return 0;
	}

	float magnitude = std::abs(static_cast<float>(input));
	float normalized = (magnitude - JOYSTICK_DEADZONE) / static_cast<float>(127 - JOYSTICK_DEADZONE);
	normalized = std::max(0.0f, std::min(1.0f, normalized));

	float curved = std::pow(normalized, DRIVE_CURVE_EXPONENT);
	int output = static_cast<int>(curved * 127.0f + 0.5f);
	if (input < 0) {
		output *= -1;
	}
	return output;
}

/**
 * Limits how fast the drivetrain can change from one command to the next.
 *
 * This helps the robot feel smoother and reduces abrupt acceleration spikes that
 * can make the chassis feel unstable.
 *
 * @param current The previous command already being sent to the motor.
 * @param target The desired command from the joystick.
 * @return A ramped value limited by MAX_SLEW_STEP.
 */
float apply_slew_limit(float current, float target) {
	float delta = target - current;
	if (delta > MAX_SLEW_STEP) {
		delta = MAX_SLEW_STEP;
	}
	if (delta < -MAX_SLEW_STEP) {
		delta = -MAX_SLEW_STEP;
	}
	return current + delta;
}

/**
 * Sends a processed left/right command to the drivetrain.
 *
 * This function applies deadzone, exponential curve, and slew limiting before
 * updating the motors. It keeps the driver feel consistent even as the driver
 * input changes quickly.
 *
 * @param left_command Raw left-side joystick input.
 * @param right_command Raw right-side joystick input.
 */
void set_drive_power(int left_command, int right_command) {
	int left_processed = apply_exponential_curve(apply_deadzone(left_command));
	int right_processed = apply_exponential_curve(apply_deadzone(right_command));

	previous_left_output = apply_slew_limit(previous_left_output, left_processed);
	previous_right_output = apply_slew_limit(previous_right_output, right_processed);

	left_mg.move(static_cast<int>(previous_left_output));
	right_mg.move(static_cast<int>(previous_right_output));
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
 * This function initializes the brain screen and registers the button callback
 * used to switch drive modes. It also configures the drivetrain brake behavior so
 * the robot stops in a predictable way when the driver releases the sticks.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::clear();
	pros::lcd::set_text(0, "Team 41658E");
	pros::lcd::set_text(1, "Driver Select");
	pros::lcd::set_text(2, "Center = Toggle");
	update_drive_mode_ui();

	left_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	right_mg.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	pros::lcd::register_btn1_cb(on_center_button);
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
 * In this robot, the drivetrain uses a simple toggle between:
 * - arcade drive: one joystick to drive and turn
 * - tank drive: left and right joysticks for each side
 *
 * We intentionally keep the drive logic in this function focused only on control
 * and not on subsystem-specific logic such as intake or lift.
 */
void opcontrol() {
	pros::Controller master(pros::E_CONTROLLER_MASTER);

	while (true) {
		int left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int right_y = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
		int right_x = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		if (current_drive_mode == DriveMode::ARCADE) {
			int forward = left_y;
			int turn = right_x;
			set_drive_power(forward - turn, forward + turn);
		} else {
			set_drive_power(left_y, right_y);
		}

		pros::delay(20);
	}
}