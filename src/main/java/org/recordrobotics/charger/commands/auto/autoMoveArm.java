package org.recordrobotics.charger.commands.auto;

import org.recordrobotics.charger.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class autoMoveArm extends CommandBase{
    
    private Arm _arm;
    private double _speed;
    private double _targetX;
    private double _targetY;

    public autoMoveArm(Arm arm, double speed, double targetX, double targetY){
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
		_arm.moveAngles(_speed, _arm.getAnglesOfRotation(_targetX, _targetY));
	}

    @Override
    public boolean isFinished() {
		return _targetX == _arm.getAnglesOfRotation(_targetX, _targetY)[0] && _targetY == _arm.getAnglesOfRotation(_targetX, _targetY)[1];
	}
}