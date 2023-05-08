package org.recordrobotics.Mitocondrion.control;

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
	 * indicates which way the arm angle should change
	 */
	enum ChangeAngle {
		INCREASE(1),
		DECREASE(-1),
		REMAIN(0);
		
		private final int _value;

		private ChangeAngle(int value) {
			_value = value;
		}

		public int value() {
			return _value;
		}
	}

	/**
	 * indicates which way the arm angle should change
	 */
	enum changeSetPointX {
		FORWARD(1),
		BACK(-1),
		REMAIN(0);
		
		private final int _value;

		private changeSetPointX(int value) {
			_value = value;
		}

		public int value() {
			return _value;
		}
	}

	enum changeSetPointY {
		UP(1),
		DOWN(-1),
		REMAIN(0);
		
		private final int _value;

		private changeSetPointY(int value) {
			_value = value;
		}

		public int value() {
			return _value;
		}
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
	 * changes the origin motor angle
	 * 
	 * @return ChangeAngle.INCREASE - increase; ChangeAngle.DECREASE - decrease; ChangeAngle.REMAIN - no change
	 */
	ChangeAngle changeOriginAngle();

	/**
	 * changes the change motor angle
	 * 
	 * @return ChangeAngle.INCREASE - increase; ChangeAngle.DECREASE - decrease; ChangeAngle.REMAIN - no change
	 */
	ChangeAngle changeChangeAngle();

/**
	 * changes set X value
	 * 
	 * @return ChangeAngle.FORWARD - forward; ChangeAngle.BACKWARD - backward; ChangeAngle.REMAIN - no change
	 */
	changeSetPointX changeSetPointX();

/**
	 * changes the change motor angle
	 * 
	 * @return ChangeAngle.UP - up; ChangeAngle.DOWN - down; ChangeAngle.REMAIN - no change
	 */
	changeSetPointY changeSetPointY();


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

	int compArm();

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
