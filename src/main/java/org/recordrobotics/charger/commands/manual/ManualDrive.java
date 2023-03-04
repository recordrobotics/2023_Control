// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger.commands.manual;

import org.recordrobotics.charger.control.IControlInput;
import org.recordrobotics.charger.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Drive teleop command
 */
public class ManualDrive extends CommandBase {

	private static final double HIGH_SPEED_MODIFIER = 0.8;
	private static final double LOW_SPEED_MODIFIER = 0.55;
	private double _speedModifier;

	private Drive _drive;
	private IControlInput _controls;

	public ManualDrive(Drive drive, IControlInput controls) {
		if (drive == null) {
			throw new IllegalArgumentException("Drive is null");
		}
		if (controls == null) {
			throw new IllegalArgumentException("Controls is null");
		}

		_drive = drive;
		_controls = controls;
		addRequirements(drive);
	}

	@Override
	public void execute() {
		if (_controls.isSlow()) {
			_speedModifier = HIGH_SPEED_MODIFIER;
		} else {
			_speedModifier = LOW_SPEED_MODIFIER;
		}
		if (_controls.canTurn()) {
			_drive.move(_controls.getDriveLong() * _speedModifier,
				_controls.getDriveLat() * _speedModifier);
		} else {
			_drive.move(_controls.getDriveLong() * _speedModifier, 0);
		}
	}

	@Override
	public void end(boolean interrupted) {
		_drive.move(0, 0);
	}

}
