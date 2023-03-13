package org.recordrobotics.charger;

/**
 * Hardware ports for computer and robot
 */
public class RobotMap {

	/**
	 * Arm Ports (Falcon 500)
	 * will change once decided
	 * Ports will also be in SIM/Neo - talk to Jake for more info
	 */
	public class Arm {
		/** the motor that will be (0,0) in the calculation */
		public static final int ORIGIN_MOTOR_PORT = 8;
		/** the motor that will change coords in the calculation */
		public static final int CHANGE_MOTOR_PORT = 7;
	}

	/**
	 * Control ports (PC USB)
	 */
	public class Control {
		// SingleControl
		public static final int SINGLE_GAMEPAD = 0;

		// DoubleControl
		public static final int DOUBLE_GAMEPAD_1 = 0;
		public static final int DOUBLE_GAMEPAD_2 = 1;
	}

	/**
	 * Drive Ports (Falcon 500)
	 */
	public class DriveBase {
		// Left
		// TODO: was 2
		public static final int LEFT_BACK_MOTOR_PORT = 9;
		public static final int LEFT_MIDDLE_MOTOR_PORT = 3;
		public static final int LEFT_FRONT_MOTOR_PORT = 1;

		// Right
		public static final int RIGHT_BACK_MOTOR_PORT = 5;
		public static final int RIGHT_MIDDLE_MOTOR_PORT = 6;
		public static final int RIGHT_FRONT_MOTOR_PORT = 4;
	}

	/**
	 * Claw Ports
	 */
	public class Claw {
		// DIO port
		public static final int LIMIT_SWITCH = 9;

		// NEO 550
		// TODO: was 9
		public static final int MOTOR_PORT = 2;
	}

}
