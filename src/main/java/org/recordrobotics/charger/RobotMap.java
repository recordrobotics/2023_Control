package org.recordrobotics.charger;

/**
 * Hardware ports for computer and robot
 */
public class RobotMap {
	/**
	 * Control ports (PC USB)
	 */
	public class Control {
		// LegacyControl
		public static final int LEGACY_GAMEPAD = 0;

		// DoubleControl
		public static final int DOUBLE_GAMEPAD_1 = 0;
		public static final int DOUBLE_GAMEPAD_2 = 1;
	}

	/**
	 * Drive Ports (Phoenix) - will change once decided
	 */
	public class DriveBase {
		public static final int LEFT_BACK_MOTOR_PORT = 2;
		public static final int LEFT_MIDDLE_MOTOR_PORT = 0;
		public static final int LEFT_FRONT_MOTOR_PORT = 1;
		public static final int RIGHT_BACK_MOTOR_PORT = 4;
		public static final int RIGHT_MIDDLE_MOTOR_PORT = 0;
		public static final int RIGHT_FRONT_MOTOR_PORT = 3;
	}

	public class Claw {
		public static final int MOTOR_PORT = 2;
	}
}
