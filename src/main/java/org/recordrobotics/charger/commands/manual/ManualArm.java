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
		// TODO: Set actual cartesian coords for ALL POSITIONS
		double[] angles;
		switch (_controls.getArmPosition()) {
			case SECOND:
				angles = _arm.getRelatedAngles(_second);
				break;
			case THIRD:
				angles = _arm.getRelatedAngles(_third);
				break;
			case SUBSTATION: // B button
				angles[0] = -35;
				angles[1] = 35;
				break;
			case GROUND://How far away must we be, A button
				angles[0] = -20;
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
				angles[0] = -20;
				angles[1] = 10;
				break;
		}
		SmartDashboard.putNumber("command set origin", angles[0]);
		_arm.setAngles(angles);
    }
}
