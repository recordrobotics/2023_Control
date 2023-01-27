package org.recordrobotics.charger.commands.auto;


import org.recordrobotics.charger.subsystems.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoMoveDistance extends CommandBase {
	private Drive _drive;
	private double _speed;
	private double _targetDistance;
	private Direction _direction;

	public AutoMoveDistance(Drive drive, double speed, double targetDistance) {
		if (drive == null) {
			throw new IllegalArgumentException("Drive is null");
		}
		if (speed <= 0) {
			throw new IllegalArgumentException("Speed must be positive");
		}
		_drive = drive;

		_speed = speed;
		_targetDistance = targetDistance;
	}

	/**
	 * resets encoders and starts move
	 */
	@Override
	public void initialize() {
		_drive.resetEncoders();
		_direction = _targetDistance > 0 ? Direction.FORWARD : Direction.BACKWARD;
		_drive.move(_speed * _direction.value(), 0);
	}

	/**
	 * command ends when encoders reach or pass the target distance
	 */
	@Override
	public boolean isFinished() {
		if (_direction == Direction.FORWARD) {
			return _drive.getPosition() >= _targetDistance;
		} else {
			return _drive.getPosition() <= _targetDistance;
		}
	}
	/**
	 * Stops the wheels once finished
	 */
	@Override
	public void end(boolean interrupted) {
		_drive.move(0, 0);
	}
}
