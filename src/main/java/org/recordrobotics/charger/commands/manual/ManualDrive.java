// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger.commands.manual;

import java.util.Map;

import org.recordrobotics.charger.control.IControlInput;
import org.recordrobotics.charger.control.IControlInput.SpeedState;
import org.recordrobotics.charger.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Drive teleop command
 */
public class ManualDrive extends CommandBase {

	private static final Map<SpeedState, Double> MODIFIERS = Map.of(
		SpeedState.FAST, 0.8,
		SpeedState.NEUTRAL, 0.55,
		SpeedState.SLOW, 0.35
	);

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
		double speedMod = MODIFIERS.get(_controls.speedState());
		if (_controls.canTurn()) {
			_drive.move(_controls.getDriveLong() * speedMod,
				_controls.getDriveLat() * speedMod);
		} else {
			_drive.move(_controls.getDriveLong() * speedMod, 0);
		}
	}

	@Override
	public void end(boolean interrupted) {
		_drive.move(0, 0);
	}

}
