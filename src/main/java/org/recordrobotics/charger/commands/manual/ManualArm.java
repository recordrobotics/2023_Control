package org.recordrobotics.charger.commands.manual;

import org.recordrobotics.charger.subsystems.Arm;
import org.recordrobotics.charger.control.IControlInput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualArm extends CommandBase {
	private Arm _arm;
	private IControlInput _controls;
	private double _speed = 0.1;
	private PIDController _originPid;
	private PIDController _changePid;
	private double _okp = 0.01;
	private double _oki;
	private double _okd;
	private double _ckp = 0.01;
	private double _cki;
	private double _ckd;
	private double _changeTolerance = 5;
	private double _originTolerance = 5;
	private double _maxSpeed = 0.5;

	public ManualArm(Arm arm, IControlInput controls, PIDController originPid, PIDController changePid) {
		if (arm == null) {
			throw new IllegalArgumentException("Arm is null");
		}
		if (controls == null) {
			throw new IllegalArgumentException("Controls is null");
		}

		_arm = arm;
		_controls = controls;
		_originPid = originPid;
		_originPid.setD(_okd);
		_originPid.setI(_oki);
		_originPid.setP(_okp);
		_originPid.setTolerance(_originTolerance);
		_changePid = changePid;
		_changePid.setD(_ckd);
		_changePid.setI(_cki);
		_changePid.setP(_ckp);
		_changePid.setTolerance(_changeTolerance);
		addRequirements(arm);
	}

	@Override
	public void execute() {
		// sets arm motor angles based on which actions is needed
		// TODO: Set actual cartesian coords for ALL POSITIONS
		double[] angles;
		switch (_controls.getArmPosition()) {
			case SECOND:
				angles = _arm.getRelatedAngles(30);
				break;
			case THIRD:
				angles = _arm.getRelatedAngles(22);
				break;
			case GROUND:
				angles = _arm.getAnglesOfRotation(40, 42);
				break;
			case SUBSTATION:
				angles = _arm.getRelatedAngles(28);
				break;
			default:
				angles = _arm.resetPositions();
				break;
		}
		_originPid.setSetpoint(angles[0]);
		_changePid.setSetpoint(angles[1]);
		double _originSpeed = _originPid.calculate(_arm.getOriginEncoder());
		double _changeSpeed = _changePid.calculate(_arm.getChangeEncoder());
		if(Math.abs(_originSpeed) > _maxSpeed){
			_originSpeed = _maxSpeed * Math.signum(_originSpeed);
		}
		if(Math.abs(_changeSpeed) > _maxSpeed){
			_changeSpeed = _maxSpeed * Math.signum(_changeSpeed);
		}

		_arm.spinOrigin(_originSpeed);
		_arm.spinChange(_changeSpeed);
	}

	@Override
	public void end(boolean interrupted) {
		// sets arm back to 0
		_arm.moveAngles(_speed, _arm.getAnglesOfRotation(0, 0));

	}
}
