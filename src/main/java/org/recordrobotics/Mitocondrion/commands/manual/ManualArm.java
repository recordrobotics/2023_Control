package org.recordrobotics.Mitocondrion.commands.manual;

import org.apache.commons.lang3.ObjectUtils.Null;
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

	public final static double maX = Units.inchesToMeters(40 - tolerance);
	public final static double maxY = Units.inchesToMeters(78 - tolerance) - Arm.ARM_BASE_HEIGHT;
	private static final double DEFAULT = -1;

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
		double[] pos = {0, 0};
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
				pos[0] = _arm.getPos(_arm.getAnglesRestricted(Units.inchesToMeters(37.375 + tolerance)))[0];
				pos[1] = Units.inchesToMeters(37.375 + tolerance) - Arm.ARM_BASE_HEIGHT;
				//angles[1] = 35;
				break;
			/*case GROUND://How far away must we be, A button
				angles[0] = -55;
				angles[1] = -50;
				break;
			*/
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
				pos[0] = DEFAULT;
				pos[1] = DEFAULT;
				_changeOffset = 0;
				break;
		}

		if(pos[0] != DEFAULT && pos[1] != DEFAULT){
			switch (_controls.changeSetPointX()){
				case FORWARD:
					if(pos[0] < maX){
						pos[0]++;
					}
					break;
				case BACK:
					pos[0]--;
				default:
					break;
			}
	
			switch (_controls.changeSetPointY()){
				case UP:
					if(pos[1] < maxY){
						pos[1]++;
					}
					break;
				case DOWN:
					pos[1]--;
				default:
					break;
			}
			angles = _arm.getAngles(pos[0], pos[1], "L");
		} else {
			angles[0] = 10;
			angles[1] = 10;
		}
		_changeOffset += 5 * _controls.changeChangeAngle().value();

		angles[1] += _changeOffset;

		SmartDashboard.putNumber("command set origin", angles[0]);
		_arm.setAngles(angles);
    }
}
