package org.recordrobotics.charger.commands.manual;

import org.recordrobotics.charger.control.IControlInput;
import org.recordrobotics.charger.subsystems.CompArm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CompManualArm extends CommandBase{
    private CompArm _arm;
	private IControlInput _controls;

    private double angles[] = {0};
	private double zero[] = {0};
    private double _speedMod = 1;

    public CompManualArm(CompArm arm, IControlInput controls) {
		if (arm == null) {
			throw new IllegalArgumentException("Arm is null");
		}
		if (controls == null) {
			throw new IllegalArgumentException("Controls is null");
		}

		_arm = arm;
		_controls = controls;

		angles = zero;		

		addRequirements(arm);
	}

    @Override
    public void execute() {
        // TODO Auto-generated method stub
		angles[0] += _controls.compArm() * _speedMod;
        _arm.setAngles(angles);
    }
}
