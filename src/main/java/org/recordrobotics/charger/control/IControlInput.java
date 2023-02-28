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
		CUBE,
		NEUTRAL,
		CONE
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
	 * Claw Turn Direction(Open & Close)
	 *
	 * @return 1 Opening, -1 Closing, 0 No Movment
	 */
	int getClawTurn();
}
