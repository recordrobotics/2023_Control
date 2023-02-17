package org.recordrobotics.charger.commands.auto;

import org.recordrobotics.charger.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoMoveArm extends CommandBase {

	private Arm _arm;
	private double _speed;
	private double _targetX;
	private double _targetY;
	private double[] _angles;

	public AutoMoveArm(Arm arm, double speed, double targetX, double targetY){
		if (speed <= 0) {
			throw new IllegalArgumentException("Speed must be positive");
		}
		if (arm == null) {
			throw new IllegalArgumentException("Arm is null");
		}

		_arm = arm;
		_targetX = targetX;
		_targetY = targetY;
		_speed = speed;
		addRequirements(arm);
	}

	@Override
	public void initialize() {
		_angles = _arm.getAnglesOfRotation(_targetX, _targetY);
		_arm.moveAngles(_speed, _angles);
	}

	@Override
	public boolean isFinished() {
		return _targetX == _angles[0] && _targetY == _angles[1];
	}
}
