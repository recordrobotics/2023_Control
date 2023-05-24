// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.Mitochondrion.commands.manual;

import org.recordrobotics.Mitochondrion.control.IControlInput;
import org.recordrobotics.Mitochondrion.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Drive teleop command
 */
public class ManualDrive extends CommandBase {

	private static final double HIGH_SPEED_MODIFIER = 0.8;
	private static final double MID_SPEED_MODIFIER = 0.55;
	private static final double SLOW_SPEED_MODIFIER = 0.35;
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
		switch (_controls.speedState()) {
			case FAST:
				_speedModifier = HIGH_SPEED_MODIFIER;
				break;
			case SLOW:
				_speedModifier = SLOW_SPEED_MODIFIER;
				break;
			case NEUTRAL:
				_speedModifier = MID_SPEED_MODIFIER;
				break;
			default:
				_speedModifier = 0;
				break;
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
