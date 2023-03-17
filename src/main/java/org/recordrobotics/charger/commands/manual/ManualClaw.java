package org.recordrobotics.charger.commands.manual;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.recordrobotics.charger.subsystems.Claw;
import org.recordrobotics.charger.control.IControlInput;


public class ManualClaw extends CommandBase {

	private Claw _claw;
	private IControlInput _controls;

<<<<<<< HEAD
	private static final double NEUTRAL_POS = -0.15;
	private static final double CUBE_POS = -0.1;
	private static final double CONE_POS = 0.1;

	private static final double TURN_SPEED = 0.05;
	// TODO: find a good value experimentally
	private static final double ERROR_MARGIN = 0.01;
=======
	//private static final double TURN_SPEED = 0.3;
>>>>>>> 076f860ed7bc098ed1ce03a391e0afb85b5722bd

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
<<<<<<< HEAD
		double currentPos = _claw.getPosition();
		double target;
		switch (_controls.getClawTurn()) {
			case CUBE:
				target = CUBE_POS;
				_claw.turn(TURN_SPEED);
				break;
			case CONE:
				target = CONE_POS;
				break;
			default:
				target = NEUTRAL_POS;
				break;
			}

		// Floats do not compare cleanly
		if (near(currentPos, target)) {
			_claw.turn(0);
		} else if (currentPos < target) {
			_claw.turn(TURN_SPEED);
		} else {
			_claw.turn(-TURN_SPEED);
		}
=======
		_claw.turn(_controls.getDriveLat());
>>>>>>> 076f860ed7bc098ed1ce03a391e0afb85b5722bd
	}


	@Override
	public void end(boolean interrupted) {
		_claw.turn(0);
	}
<<<<<<< HEAD

	/**
	 * Check if two doubles are close enough to be considered equal
	 *
	 * @param num0 First number.
	 * @param num1 Second number.
	 * @return true - near equal, false - not equal
	 */
	private boolean near(double num0, double num1) {
		return Math.abs(num0 - num1) < ERROR_MARGIN;
	}
=======
>>>>>>> 076f860ed7bc098ed1ce03a391e0afb85b5722bd
}
