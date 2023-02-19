package org.recordrobotics.charger.commands.manual;

import org.recordrobotics.charger.control.IControlInput;
import org.recordrobotics.charger.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * Drive teleop command
 */
public class ManualDrive extends CommandBase {

	private static final double SPEED_MODIFIER = 0.5;

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
		_drive.move(_controls.getDriveLong() * SPEED_MODIFIER,
			_controls.getDriveLat() * SPEED_MODIFIER);
	}

	@Override
	public void end(boolean interrupted) {
		_drive.move(0, 0);
	}

}
