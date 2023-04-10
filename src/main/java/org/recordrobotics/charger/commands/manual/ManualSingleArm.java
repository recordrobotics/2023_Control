package org.recordrobotics.charger.commands.manual;

import org.recordrobotics.charger.control.IControlInput;
import org.recordrobotics.charger.subsystems.Arm;
import org.recordrobotics.charger.subsystems.SingleSegmentArm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualSingleArm extends CommandBase{
	private SingleSegmentArm _arm;
	private IControlInput _controls;

	private double _changeOffset;

    private double[] angles = {0,0};

    public ManualSingleArm(SingleSegmentArm arm, IControlInput controls, PIDController changePid) {
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
		//angle = {0, 0};
		switch (_controls.changeChangeAngle()) {//TODO: get better heights, and maybe differentiate between cubes and cones
			case INCREASE: 
				angles[1] += 0.3;
				break;
			case DECREASE:
				angles[1] += -0.3;
				break;
			case REMAIN: // B button
				//angles[1] += 0;
				break;

			default:
				//angles[1] = 10;
				//_changeOffset = 0;
				break;
		}

		_changeOffset += 5 * _controls.changeChangeAngle().value();
		angles[1] += _changeOffset;

		SmartDashboard.putNumber("command set origin", angles[0]);

		_arm.setAngles(angles);
    }
}
