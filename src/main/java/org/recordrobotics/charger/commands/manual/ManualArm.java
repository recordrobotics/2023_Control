package org.recordrobotics.charger.commands.manual;

import org.recordrobotics.charger.subsystems.Arm;
import org.recordrobotics.charger.control.IControlInput;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualArm extends CommandBase {
	private Arm _arm;
	private IControlInput _controls;
	private static double speed = 0.1;

	public ManualArm(Arm arm, IControlInput controls) {
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
				angles = _arm.getRelatedAngles(30);
				break;
			case THIRD:
				angles = _arm.getRelatedAngles(42);
				break;
			case GROUND:
				angles = _arm.getAnglesOfRotation(40, 42);
				break;
			case SUBSTATION:
				angles = _arm.getRelatedAngles(42);
				break;
			default:
				angles = _arm.resetPositions();
				break;
		}
		_arm.moveAngles(speed, angles);
	}

	@Override
	public void end(boolean interrupted) {
		// sets arm back to 0
		_arm.moveAngles(speed, _arm.getAnglesOfRotation(0, 0));

	}
}
