package org.recordrobotics.charger.commands.auto;

import org.recordrobotics.charger.subsystems.Arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ArmHoldPosition extends CommandBase{
	private PIDController _pid;
	private double _tolerance = 3;
	private Arm _arm;
	private double[] _target;
	private boolean _clawDone;

	@SuppressWarnings({"PMD.UseVarargs","PMD.ArrayIsStoredDirectly"})
	public ArmHoldPosition(Arm arm, double[] target){
		_pid = new PIDController(0, 0, 0);
		_arm = arm;
		_target = target;
		_pid.setTolerance(_tolerance);
		addRequirements(arm);
	}

	@Override
	public void execute() {
		double _originSpeed = _pid.calculate(_arm.getOriginEncoder(), _target[0]);
		double _changeSpeed = _pid.calculate(_arm.getChangeEncoder(), _target[1]);
		_arm.spinOrigin(_originSpeed);
		_arm.spinChange(_changeSpeed);
	}

	@Override
	public boolean isFinished(){
		return _clawDone;
	}

	@Override
	public void end(boolean interrupted){
		_arm.spinOrigin(0);
		_arm.spinChange(0);
	}
}
