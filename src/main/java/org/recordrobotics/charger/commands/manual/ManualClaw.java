package org.recordrobotics.charger.commands.manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.recordrobotics.charger.subsystems.Claw;
import org.recordrobotics.charger.control.IControlInput;

public class ManualClaw extends CommandBase {

	private Claw _claw;
	private IControlInput _controls;

	private static final double TURN_SPEED = 0.2;
	private static final double MAX = 0.1;
	// TODO: find a good value experimentally

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
			case OPEN:
				if (_claw.getPosition() < MAX) {
					_claw.turn(TURN_SPEED);
				} else {
					_claw.turn(0);
				}
				break;
			case CLOSE:
				_claw.turn(-TURN_SPEED);
				break;
			case NEUTRAL:
				_claw.turn(0);
				break;
			}
	}

	@Override
	public void end(boolean interrupted) {
		_claw.turn(0);
	}

	/**
	 * Check if two doubles are close enough to be considered equal
	 *
	 * @param num0 First number.
	 * @param num1 Second number.
	 * @return true - near equal, false - not equal
	 */
}
