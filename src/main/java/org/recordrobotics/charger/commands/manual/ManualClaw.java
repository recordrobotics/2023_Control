package org.recordrobotics.charger.commands.manual;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import org.recordrobotics.charger.subsystems.Claw;
import org.recordrobotics.charger.control.IControlInput;

public class ManualClaw extends CommandBase {

	private Claw _claw;
	private IControlInput _controls;
	private PIDController _pid;
	private double _tolerance = 3.0;

	private static final double NEUTRAL_POS = 0.3;
	private static final double CUBE_POS = 0.1;
	private static final double CONE_POS = 0.0;

	public ManualClaw(Claw claw, IControlInput controls) {
		if (claw == null) {
			throw new IllegalArgumentException("claw is null");
		}
		if (controls == null) {
			throw new IllegalArgumentException("Controls is null");
		}

		_claw = claw;
		_controls = controls;
		_pid = new PIDController(0, 0, 0);
		_pid.setTolerance(_tolerance);
		addRequirements(claw);
	}

	@Override
	public void execute() {
		double target;
		switch (_controls.getClawTurn()) {
			case CUBE:
				target = CUBE_POS;
				break;
			case CONE:
				target = CONE_POS;
				break;
			default:
				target = NEUTRAL_POS;
				break;
		}
		_claw.turn(_pid.calculate(_claw.getPosition(), target));
	}

	public boolean isFinished(){
		return _pid.atSetpoint();
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
