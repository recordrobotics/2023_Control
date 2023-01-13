// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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
}
