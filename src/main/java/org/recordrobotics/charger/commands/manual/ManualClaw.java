package org.recordrobotics.charger.commands.manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.recordrobotics.charger.subsystems.Claw;
import org.recordrobotics.charger.control.IControlInput;


public class ManualClaw extends CommandBase {

	private Claw _claw;
	private IControlInput _controls;

	public ManualClaw(Claw claw, IControlInput controls) {
		if (claw == null) {
			throw new IllegalArgumentException("claw is null");
		}
		if (controls == null) {
			throw new IllegalArgumentException("Controls is null");
		}

		_claw = claw;
		_controls = controls;
		addRequirements(claw);
	}

	@Override
	public void execute() {
		if(_controls.getClawOpen()){
			_claw.turn(0.3);
		}else if(_controls.getClawClose()){
			_claw.turn(-0.3);
		}
	}

	@Override
	public void end(boolean interrupted) {
		_claw.turn(0);
	}
}
