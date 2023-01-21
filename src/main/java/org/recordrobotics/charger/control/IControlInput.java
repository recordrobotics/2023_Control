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
	 * Go to second level in treasury
	 *
	 * @return true - do it, false - don't do it
	 */
	boolean moveToSecond();

	/**
	 * Go to third level in treasury
	 *
	 * @return true - do it, false - don't do it
	 */
	boolean moveToThird();

	
}
