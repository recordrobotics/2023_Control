package org.recordrobotics.charger.commands.manual;

import org.recordrobotics.charger.Constants;
import org.recordrobotics.charger.control.IControlInput;
import org.recordrobotics.charger.subsystems.Arm2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualArm2 extends CommandBase{
    private Arm2 _arm;
	private IControlInput _controls;

    public ManualArm2(Arm2 arm, IControlInput controls, PIDController originPid, PIDController changePid) {
		if (arm == null) {
			throw new IllegalArgumentException("Arm is null");
		}
		if (controls == null) {
			throw new IllegalArgumentException("Controls is null");
		}

		_arm = arm;
		_controls = controls;
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
		_arm.setAngles(angles);
    }
}
