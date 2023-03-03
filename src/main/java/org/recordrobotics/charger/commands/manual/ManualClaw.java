package org.recordrobotics.charger.commands.manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.recordrobotics.charger.subsystems.Claw;
import org.recordrobotics.charger.control.IControlInput;


public class ManualClaw extends CommandBase {

	private Claw _claw;
	private IControlInput _controls;

	private static final double TURN_SPEED = 0.05;

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
	public void initialize() {
		_claw.resetEncoders();
	}

	@Override
	public void execute() {
		switch (_controls.getClawTurn()) {
			case 1:
				_claw.turn(TURN_SPEED);
				break;
			case -1:
				if (_claw.getCurrent() < _claw._CURRENT_GRAB_THRESHOLD) {
					_claw.turn(-TURN_SPEED);
				} else {
					_claw.turn(0);
				}
				break;
			default:
			 	_claw.turn(0);
				break;
			}
	}


	@Override
	public void end(boolean interrupted) {
		_claw.turn(0);
	}

}
