// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * Put constants here
 */
public final class Constants {
	public static final String COMMANDS_TAB = "commands";
	public static final String DATA_TAB = "data";
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

	public class FieldElements {//TODO: verify these values
		public static final double DISTANCE_TO_FAR_NODE = 1.016;
		public static final double CUBE_MID_HEIGHT = 0.523113;
		public static final double CUBE_TOP_HEIGHT = 0.826389;
		public static final double SUBSTATION_HEIGHT = 0.949327;
	}


	public static class DriveConstants  {
		public static final double ksVolts = 8;
		public static final double kvVoltSecondsPerMeter = 4;
		public static final double kaVoltSecondsSquaredPerMeter = 2;
		public final static DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22));
	}

	public static class AutoConstants {
		public static final double kMaxSpeedMetersPerSecond = 3;
		public static final double kMaxAccelerationMetersPerSecondSquared = 3;

	}
	
	

}
