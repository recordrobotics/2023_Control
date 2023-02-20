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
		// TODO: Set actual angle values for ALL POSITIONS
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
		_arm.moveAngles(speed, angles);
		System.out.println("change " + _arm.getChangeEncoder());
		System.out.println("origin " + _arm.getOriginEncoder());
	}

	@Override
	public void end(boolean interrupted) {
		// sets arm back to 0
		double[] angles;
		angles= _arm.getAnglesOfRotation(0, 0);
		_arm.moveAngles(speed, angles);
		
	}
}
