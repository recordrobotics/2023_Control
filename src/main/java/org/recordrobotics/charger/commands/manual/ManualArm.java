package org.recordrobotics.charger.commands.manual;

import org.recordrobotics.charger.subsystems.Arm;
import org.recordrobotics.charger.control.IControlInput;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualArm extends CommandBase {
	private Arm _arm;
	private IControlInput _controls;

	public ManualArm(Arm arm, IControlInput controls) {
		if (arm == null) {
			throw new IllegalArgumentException("Drive is null");
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
		double[] angles;
		if (_controls.moveToSecond()) {
			angles = _arm.getAnglesOfRotation(23, 30);
		} else if (_controls.moveToThird()) {
			angles = _arm.getAnglesOfRotation(40, 42);
		} else if (_controls.pickUpFromGround()) {
			angles = _arm.getAnglesOfRotation(40, 42);
		} else if (_controls.pickUpFromSub()) {
			angles = _arm.getAnglesOfRotation(40, 42);
		} else {
			angles = _arm.resetPositions();
		}
		_arm.moveAngles(angles);
	}

	@Override
	public void end(boolean interrupted) {
		_arm.getAnglesOfRotation(0, 0);
	}
}
