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
	 * Determines the speed the robot drives at
	 *
	 * @return true - lower speed; false - faster speed
	 */
	boolean isSlow();

	/**
	 * Enables/disables left and right turning
	 *
	 * @return true - can turn; false - cannot turn
	 */
	boolean canTurn();
}
