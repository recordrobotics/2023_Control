package org.recordrobotics.charger.commands.dashboard;

import org.recordrobotics.charger.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DashResetArmEncoders extends CommandBase {

	private Arm _arm;

	public DashResetArmEncoders(Arm arm) {
		if (arm == null) {
			throw new IllegalArgumentException("Climbers is null");
		}

		_arm = arm;
	}

	/**
	 * Reset climber encoder values when started
	 */
	@Override
	public void initialize() {
		_arm.resetEncoders();
	}

	/**
	 * Command finished immediately
	 */
	@Override
	public boolean isFinished() {
		return true;
	}

}
