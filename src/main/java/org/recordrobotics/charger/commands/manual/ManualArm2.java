package org.recordrobotics.charger.commands.manual;

import org.recordrobotics.charger.Constants;
import org.recordrobotics.charger.control.IControlInput;
import org.recordrobotics.charger.subsystems.Arm2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualArm2 extends CommandBase{
    private Arm2 _arm;
	private IControlInput _controls;

	private PIDController _originPid;
	private PIDController _changePid;
	private double _okp = 0;
	private double _oki;
	private double _okd;
	private double _ckp = 0;
	private double _cki;
	private double _ckd;
	private double _changeTolerance = 2.5;
	private double _originTolerance = 2.5;
	private double _maxSpeed = 0.3;
	//private double _maxDownSpeed = 0.15;


    public ManualArm2(Arm2 arm, IControlInput controls, PIDController originPid, PIDController changePid) {
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
		double[] angles;
		switch (_controls.getArmPosition()) {//TODO: get better heights, and maybe differentiate between cubes and cones
			case SECOND:
				angles = _arm.getAnglesRestricted(Arm2.FIRST_ARM_LENGTH, Arm2.SECOND_ARM_LENGTH, 0.9);
				break;
			case THIRD:
				angles = _arm.getAngles(Arm2.FIRST_ARM_LENGTH, Arm2.SECOND_ARM_LENGTH, Constants.FieldElements.DISTANCE_TO_FAR_NODE, Constants.FieldElements.CUBE_TOP_HEIGHT, "L");
				break;
			case GROUND://How far away must we be
				angles = null;
				break;
			case SUBSTATION:
				angles = _arm.getAnglesRestricted(Arm2.FIRST_ARM_LENGTH, Arm2.SECOND_ARM_LENGTH, Constants.FieldElements.SUBSTATION_HEIGHT);
				break;
			case FLIP_GROUND_ORIGIN://What is this?
				angles = null;
				break;
			case FLIP_GROUND_CHANGE://What is this?
				angles = null;
				break;
			default:
				angles = _arm.getAngles(Arm2.FIRST_ARM_LENGTH, Arm2.SECOND_ARM_LENGTH, 1.07, 1.07, "R");//This should extend mostly fully, but not quite
				break;
		}

        _originPid.setSetpoint(angles[0]);
		_changePid.setSetpoint(angles[1]);
		double _originSpeed = _originPid.calculate(_arm.getOriginEncoder());
		double _changeSpeed = _changePid.calculate(_arm.getChangeEncoder());  
		if (_originSpeed < 0){
			_originSpeed = Math.max(_originSpeed, _maxSpeed);
		} else {
			_originSpeed = Math.min(_originSpeed, _maxSpeed);
		}
		if (_changeSpeed < 0){
			_changeSpeed = Math.max(_changeSpeed, _maxSpeed);
		} else {
			_changeSpeed = Math.min(_changeSpeed, _maxSpeed);
		}

		_arm.spinOrigin(_originSpeed);
		_arm.spinChange(_changeSpeed);
    }
}