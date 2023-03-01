package org.recordrobotics.charger.control;

/**
 * Specifies all control inputs needed for the robot
 */
public interface IControlInput {

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
	 * Returns goal of arm movement
	 *
	 * @return ArmPosition.SUBSTATION - go to substation
	 * ArmPosition.GROUND - go to ground
	 * ArmPosition.SECOND - go to second row
	 * ArmPosition.THIRD - go to third row
	 * ArmPosition.NEUTRAL - go to neutral position
	 */
	ArmPosition getArmPosition();

	public enum ArmPosition {
		SUBSTATION,
		GROUND,
		SECOND,
		THIRD,
		NEUTRAL, 
		FLIP_GROUND_ORIGIN, 
		FLIP_GROUND_CHANGE;
	}
}
