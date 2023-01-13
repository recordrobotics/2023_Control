package org.recordrobotics.charger.control;

/**
 * Specifies all control inputs needed for the robot
 */
public interface IControlInput {

	/**
	 * Flywheel has three states:
	 * 	OFF - disable
	 * 	LOW - low target shot
	 * 	HIGH - high target shot
	 */
	enum FlywheelState {
		OFF,
		LOW,
		HIGH
	}

	/**
	 * Logitudinal drive input (forward & backward) value
	 *
	 * @return [-1, 0) - backward; (0, 1] - forward
	 */
	double getDriveLong();

	/**
	 * Latitudinal drive input (left & right) value
	 *
	 * @return [-1, 0) - left; (0, 1] - right
	 */
	double getDriveLat();

	/**
	 * Rotator input value
	 *
	 * @return [-1, 0) - backward; (0, 1] - forward
	 */
	double getRotate();

	/**
	 * Climber input value
	 *
	 * @return [-1, 0) - down; (0, 1] - up
	 */
	double getClimb();

	/**
	 * Acquisition spin input value
	 *
	 * @return [-1, 0) - backward; (0, 1] - forward
	 */
	double getAcqSpin();

	/**
	 * Acquisition tilt input value
	 *
	 * @return [-1, 0) - in; (0, 1] - out
	 */
	double getAcqTilt();

	/**
	 * Flywheel requested state
	 *
	 * @return OFF - off, LOW - low speed, HIGH - high speed
	 */
	FlywheelState getFlywheel();

	/**
	 * Servo input
	 *
	 * @return false - servos down; true - servos up
	 */
	boolean getServos();

}
