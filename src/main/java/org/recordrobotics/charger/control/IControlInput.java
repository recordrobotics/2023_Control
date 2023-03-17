package org.recordrobotics.charger.control;

/**
 * Specifies all control inputs needed for the robot
 */
public interface IControlInput {

	/**
	 * Claw has three states:
	 * 	CUBE - grabs cube
	 * 	NEUTRAL - open
	 * 	CONE - grabs cone
	 */
	enum ClawState {
		OPENING,
		NEUTRAL,
		GRABING
	}

	/**
	 * Claw has three states:
	 * 	FAST - fast speed
	 * 	NEUTRAL - normal speed
	 * 	SLOW - slow speed
	 */
	enum SpeedState {
		FAST,
		NEUTRAL,
		SLOW
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
	 * Determines the speed the robot drives at
	 *
	 * @return true - lower speed; false - faster speed
	 */
	SpeedState speedState();

	/**
	 * Enables/disables left and right turning
	 *
	 * @return true - can turn; false - cannot turn
	 */
	boolean canTurn();
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
		/**
	 * Claw Turn Direction(Release & Grab)
	 *
	 * @return 1 Releasing, -1 Grabbing, 0 No Movement
	 */
	ClawState getClawTurn();
}
