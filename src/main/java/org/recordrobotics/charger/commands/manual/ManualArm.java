package org.recordrobotics.charger.commands.manual;

import org.recordrobotics.charger.subsystems.Arm;
import org.recordrobotics.charger.control.IControlInput;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

@SuppressWarnings({"PMD.TooManyFields","PMD.FieldNamingConventions"})
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
	private double _maxDownSpeed = 0.2;

	private static double _second[] = {57.58, -147.58 - (5/7 * 57.58)};
	private static double _third[] = {40.21, -139.87 - (5/7 * 40.21)};
	private static double _substation[] = {10.41, -100.41 - (5/7 * 10.41)};
	private static double _placehold[];

	/*private static double _second = 30;
	private static double _third = 22;
	private static double _ground[] = {40, 42};
	private static double _substation = 28;

	private static double _flipGroundOriginX = 22;
	private static double _flipGroundOriginY = 22;
	private static double _flipGroundChangeX = 22;
	private static double _flipGroundChangeY = 22;*/

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
				angles = _second;//_arm.getRelatedAngles(_second);
				break;
			case THIRD:
				angles = _third;//_arm.getRelatedAngles(_third);
				break;
			case GROUND:
				angles = _placehold;//_arm.getAnglesOfRotation(_ground[0], _ground[1]);
				break;
			case SUBSTATION:
				angles = _substation;//_arm.getRelatedAngles(_substation);
				break;
			case FLIP_GROUND_ORIGIN:
				angles = _placehold;//_arm.getAnglesOfRotation(_flipGroundOriginX,_flipGroundOriginY);
				break;
			case FLIP_GROUND_CHANGE:
				angles = _placehold;//_arm.getAnglesOfRotation(_flipGroundChangeX, _flipGroundChangeY);
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
			if(Math.sin(_arm.getOriginEncoder()) < Math.sqrt(2)/2){
				_originSpeed = _maxDownSpeed * Math.signum(_originSpeed);
			}else{
				_originSpeed = _maxSpeed * Math.signum(_originSpeed);
			}
		}
		if(Math.abs(_changeSpeed) > _maxSpeed){
			if(Math.sin(_arm.getChangeEncoder() + _arm.getOriginEncoder() * 5/7) < Math.sqrt(2)/2){
			_changeSpeed = _maxDownSpeed * Math.signum(_changeSpeed);
			}else{
				_changeSpeed = _maxSpeed * Math.signum(_changeSpeed);
			}
		}

		_arm.spinOrigin(_originSpeed);
		_arm.spinChange(_changeSpeed);
	}

	public boolean originIsFliped(){
		return _originPid.atSetpoint();
	}

	public boolean _changeIsFliped(){
		return _changePid.atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		// sets arm back to 0
		_arm.moveAngles(_speed, _arm.getAnglesOfRotation(0, 0));

	}
}
