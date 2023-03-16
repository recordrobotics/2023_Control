package org.recordrobotics.charger.commands.manual;

import org.recordrobotics.charger.Constants;
import org.recordrobotics.charger.control.IControlInput;
import org.recordrobotics.charger.subsystems.Arm2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ManualArm2 extends CommandBase{
    private Arm2 _arm;
	private IControlInput _controls;

	private PIDController _originPid;
	private PIDController _changePid;
	private static final double C_KP = 0.005;
	private static final double C_KI = 0.001;
	private static final double C_KD = 0.0005;
	private static final double O_KP = 0.005;
	private static final double O_KI = 0;
	private static final double O_KD = 0.0015;
	private double _changeTolerance = 0;
	private double _originTolerance = 0;
	private double _originMaxSpeed = 0.2;
	private double _changeMaxSpeed = 0.1;
	private double[] prevAngles = {0, 0};
	//private double _maxDownSpeed = 0.15;

	private GenericEntry _targetOrigin;
	private GenericEntry _currentOrigin;
	private GenericEntry _speedOfOrigin;
	private GenericEntry _targetChange;
	private GenericEntry _currentChange;
	private GenericEntry _speedOfChange;

    public ManualArm2(Arm2 arm, IControlInput controls, PIDController originPid, PIDController changePid) {
		if (arm == null) {
			throw new IllegalArgumentException("Arm is null");
		}
		if (controls == null) {
			throw new IllegalArgumentException("Controls is null");
		}

		_arm = arm;
		_arm.resetEncoders();
		_controls = controls;
		_originPid = originPid;
		_originPid.setD(O_KD);
		_originPid.setI(O_KI);
		_originPid.setP(O_KP);
		_originPid.setTolerance(_originTolerance);
		_changePid = changePid;
		_changePid.setD(C_KD);
		_changePid.setI(C_KI);
		_changePid.setP(C_KP);
		_changePid.setTolerance(_changeTolerance);
		addRequirements(arm);
		ShuffleboardTab tab = Shuffleboard.getTab(Constants.DATA_TAB);
		_targetOrigin = tab.add("Origin Target", new double[] {0}).getEntry();
		_currentOrigin = tab.add("Origin Position", new double[] {0}).getEntry();
		_speedOfOrigin = tab.add("Origin Speed", new double[] {0}).getEntry();
		_targetChange = tab.add("Change Target", new double[] {0}).getEntry();
		_currentChange = tab.add("Change Position", new double[] {0}).getEntry();
		_speedOfChange = tab.add("Change Speed", new double[] {0}).getEntry();
	}

    @Override
	public void execute() {
		// sets arm motor angles based on which actions is needed
		double[] angles = {0, 0};
		switch (_controls.getArmPosition()) {//TODO: get better heights, and maybe differentiate between cubes and cones
			case SECOND: // X Button
				angles[0] = -60;
				angles[1] = 0;
				break;
			case THIRD: // Y button
				angles[0] = -60;
				angles[1] = 30;
				break;
			case SUBSTATION: // B button
				angles[0] = -60;
				angles[1] = 30;
				break;
			case GROUND://How far away must we be, A button
				angles[0] = -75;
				angles[1] = -10;
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

		if(angles[0] != prevAngles[0]) {
    		_originPid.setSetpoint(angles[0]);
			_targetOrigin.setDouble(angles[0]);
			SmartDashboard.putNumber("Origin Target", angles[0]);
			prevAngles[0] = angles[0];
		}	
		if(angles[1] != prevAngles[1]) {
			_changePid.setSetpoint(angles[1]);
			_targetChange.setDouble(angles[1]);
			SmartDashboard.putNumber("Change Target", angles[1]);
			prevAngles[1] = angles[1];
		}

		double originPos = _arm.getOriginEncoder();
		double changePos = _arm.getChangeEncoder();
		_currentOrigin.setDouble(originPos);
		SmartDashboard.putNumber("Origin Pos", originPos);
		_currentChange.setDouble(changePos);
		SmartDashboard.putNumber("Change Pos", changePos);
		double _originSpeed = _originPid.calculate(originPos);
		double _changeSpeed = _changePid.calculate(changePos);  
		_originSpeed = Math.min(Math.abs(_originSpeed), _originMaxSpeed) * Math.signum(_originSpeed);
		_changeSpeed = Math.min(Math.abs(_changeSpeed), _changeMaxSpeed) * Math.signum(_changeSpeed);
		_speedOfOrigin.setDouble(_originSpeed);
		SmartDashboard.putNumber("Origin Speed", _originSpeed);
		_speedOfChange.setDouble(_changeSpeed);
		SmartDashboard.putNumber("Change Speed", _changeSpeed);
		
		_arm.spinOrigin(_originSpeed);
		_arm.spinChange(_changeSpeed);

	//	System.out.println("Origin Encoder Value = " + _arm.getOriginEncoder());
	//	System.out.println("Origin Speed = " + _originSpeed);
	//	System.out.println("Origin Target = " + angles[0]);
	//	System.out.println("Change Encoder Value = " + _arm.getChangeEncoder());
	//	System.out.println("Change Speed = " + _changeSpeed);
	//	System.out.println("Change Target = " + angles[1]);
    }
}
