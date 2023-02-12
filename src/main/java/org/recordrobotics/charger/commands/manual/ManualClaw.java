package org.recordrobotics.charger.commands.manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.recordrobotics.charger.subsystems.Claw;
import org.recordrobotics.charger.control.IControlInput;


public class ManualClaw extends CommandBase {

	private Claw _claw;
	private IControlInput _controls;

	private static final double TURN_SPEED = 0.3;

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
		if(_controls.getClawTurn() > 0) {		
			if(Claw.CLAW_CUBE - _claw.getPosition() > 0) {	
				_claw.turn(TURN_SPEED);
			} else if(Claw.CLAW_CUBE - _claw.getPosition() < 0) {
				_claw.turn(-TURN_SPEED);
			} else {
				_claw.turn(0);
			}
				
		}else if(_controls.getClawTurn() < 0) {
			if(Claw.CLAW_CONE - _claw.getPosition() > 0) {	
				_claw.turn(TURN_SPEED);
			} else if(Claw.CLAW_CONE - _claw.getPosition() < 0) {
				_claw.turn(-TURN_SPEED);
			} else {
				_claw.turn(0);
			}

		}else{
			_claw.turn(-TURN_SPEED);

		}
	}

	@Override
	public void end(boolean interrupted) {
		_claw.turn(0);
	}
}
