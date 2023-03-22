package org.recordrobotics.charger.commands.manual;

import org.recordrobotics.charger.control.IControlInput;
import org.recordrobotics.charger.subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualArm extends CommandBase{
	private Arm _arm;
	private IControlInput _controls;

    public ManualArm(Arm arm, IControlInput controls, PIDController originPid, PIDController changePid) {
		if (arm == null) {
			throw new IllegalArgumentException("Arm is null");
		}
		if (controls == null) {
			throw new IllegalArgumentException("Controls is null");
		}

		_arm = arm;
		_controls = controls;

		addRequirements(arm);
	}

    @Override
	public void execute() {
		// sets arm motor angles based on which actions is needed
		double[] angles = {0, 0};
		switch (_controls.getArmPosition()) {//TODO: get better heights, and maybe differentiate between cubes and cones
			case SECOND: // X Button
				angles[0] = -30;
				angles[1] = 0;
				break;
			case THIRD: // Y button
				angles[0] = 0;
				angles[1] = 0;
				break;
			case SUBSTATION: // B button
				angles[0] = -70;
				angles[1] = 0;
				break;
			case GROUND://How far away must we be, A button
				angles[0] = 0;
				angles[1] = 0;
				break;
	//		case THIRD:
	//			angles = _arm.getAngles(Arm2.FIRST_ARM_LENGTH, Arm2.SECOND_ARM_LENGTH, Constants.FieldElements.DISTANCE_TO_FAR_NODE, Constants.FieldElements.CUBE_TOP_HEIGHT, "L");
	//			break;
	//		case GROUND://How far away must we be
	//			//angles = null; this still needs to be fixed
	//			angles = _arm.getAngles(Arm2.FIRST_ARM_LENGTH, Arm2.SECOND_ARM_LENGTH, Constants.FieldElements.DISTANCE_TO_FAR_NODE, Constants.FieldElements.CUBE_TOP_HEIGHT, "L");
	//			break;
	//		case SUBSTATION:
	//			angles = _arm.getAnglesRestricted(Arm2.FIRST_ARM_LENGTH, Arm2.SECOND_ARM_LENGTH, Constants.FieldElements.SUBSTATION_HEIGHT);
	//			break;
		//	case FLIP_GROUND_ORIGIN://What is this?
		//		angles = null;
		//		break;
		//	case FLIP_GROUND_CHANGE://What is this?
		//		angles = null;
		//		break;
			default:
				//angles = _arm.getAngles(Arm2.FIRST_ARM_LENGTH, Arm2.SECOND_ARM_LENGTH, 1.07, 1.07, "R");//This should extend mostly fully, but not quite
				break;
		}
<<<<<<< HEAD
		SmartDashboard.putNumber("command set origin", angles[0]);
		_arm.setAngles(angles);
    }
=======
		_originPid.setSetpoint(angles[0]);
		_changePid.setSetpoint(angles[1]);
		double _originSpeed = _originPid.calculate(_arm.getOriginEncoder());
		double _changeSpeed = _changePid.calculate(_arm.getChangeEncoder());
		if(Math.abs(_originSpeed) > _maxSpeed){
			if(Math.sin(Units.degreesToRadians(_arm.getOriginEncoder())) < -Math.sqrt(2)/2){
				_originSpeed = _maxDownSpeed * Math.signum(_originSpeed);
			}else{
				_originSpeed = _maxSpeed * Math.signum(_originSpeed);
			}
		}
		if(Math.abs(_changeSpeed) > _maxSpeed){
			if(Math.sin(Units.degreesToRadians(_arm.getChangeEncoder() + _arm.getOriginEncoder() * 5/7)) < -Math.sqrt(2)/2){
			_changeSpeed = _maxDownSpeed * Math.signum(_changeSpeed);
			}else{
				_changeSpeed = _maxSpeed * Math.signum(_changeSpeed);
			}
		}

		_arm.spinOrigin(_originSpeed);
		_arm.spinChange(_changeSpeed);
		//System.out.println(_arm.getOriginEncoder() + " " + angles[0]);
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
>>>>>>> second-competion-merge
}
