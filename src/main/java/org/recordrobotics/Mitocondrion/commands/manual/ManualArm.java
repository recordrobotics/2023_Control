package org.recordrobotics.Mitocondrion.commands.manual;

import org.recordrobotics.Mitocondrion.control.IControlInput;
import org.recordrobotics.Mitocondrion.subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualArm extends CommandBase{
	private Arm _arm;
	private IControlInput _controls;

	private double _changeOffset;

	public final static double tolerance = 3;

	public final static double maX = Units.inchesToMeters(24 - tolerance);
	public final static double maxY = Units.inchesToMeters(78 - tolerance) - Arm.ARM_BASE_HEIGHT;
	private static final double DEFAULT = -1;

	private double[] pos = {0, 0.7};
	private double[] angles = {-10, 10};

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

	public void resetPos() {
		pos[0] = 0;
		pos[1] = 0.7;
	}

    @Override
	public void execute() {
		// sets arm motor angles based on which actions is needed
		switch (_controls.getArmPosition()) {//TODO: get better heights, and maybe differentiate between cubes and cones
			case SECOND: // X Button -- working
				pos[0] = Units.inchesToMeters(14.25 + tolerance);
				pos[1] = Units.inchesToMeters(23.5 + tolerance) - Arm.ARM_BASE_HEIGHT;
				break;
			case THIRD: // Y button -- NEEDS TUNING
				pos[0] = Units.inchesToMeters(31.625 + tolerance);
				pos[1] = Units.inchesToMeters(35.5 + tolerance) - Arm.ARM_BASE_HEIGHT;
				//angles[0] = -45;
				//angles[1] = 30;
				break;
			case SUBSTATION: // B button
				//pos[0] = _arm.getPos(_arm.getAnglesRestricted(Units.inchesToMeters(37.375 + tolerance)))[0];
				//pos[1] = Units.inchesToMeters(37.375 + tolerance) - Arm.ARM_BASE_HEIGHT;
				pos[0] = 0.33;
				pos[1] = 0.83;
				//angles[1] = 35;
				break;
			case GROUND://How far away must we be, A button
				angles[0] = -10;
				angles[1] = 10;
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
				_changeOffset = 0;
				break;
		}

		switch (_controls.changeSetPointX()){
			case FORWARD:
					pos[0] += 0.01;
				break;
			case BACK:
				pos[0] -= 0.01;
			default:
				break;
		}

		switch (_controls.changeSetPointY()){
			case UP:
					pos[1] += 0.01;
				break;
			case DOWN:
					pos[1] -= 0.01;
			default:
				break;
		}

		pos[0] = Math.max(Math.min(pos[0], maX), 0);
		pos[1] = Math.max(Math.min(pos[1], maxY), 0);
		angles = _arm.getAngles(pos[0], pos[1], "L");
		_changeOffset += 5 * _controls.changeChangeAngle().value();

		angles[1] += _changeOffset;

		SmartDashboard.putNumber("Pos X", pos[0]);
		SmartDashboard.putNumber("Pos Y", pos[1]);
		SmartDashboard.putNumber("command set origin", angles[0]);
		_arm.setAngles(angles);
    }
}
