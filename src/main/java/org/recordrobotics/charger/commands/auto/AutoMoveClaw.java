package org.recordrobotics.charger.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.recordrobotics.charger.commands.manual.ManualClaw;
import org.recordrobotics.charger.subsystems.Claw;

public class AutoMoveClaw extends CommandBase{
	private Claw _claw;
	private double _speed;
	private int _status;

	/**
	 * @param claw Claw object
	 * @param speed speed to turn
	 * @param status grab or release, 1 is grab, -1 is release
	 */
	public AutoMoveClaw(Claw claw, double speed, int status){
		if (speed <= 0) {
			throw new IllegalArgumentException("Speed must be positive");
		}
		if (claw == null) {
			throw new IllegalArgumentException("Drive is null");
		}

		_claw = claw;
		_speed = speed;
		_status = status;
	}

	/**
	 * Open/close claw
	 */
	@Override

	public void initialize() {
		if (_status == 1) {
			if (_claw.getCurrent() > 5.0 || _claw.getSwitchState()) {
				_claw.turn(0);
			} else {
				_claw.turn(-_speed);
			}
		} else {
			_claw.turn(_speed);
		}
	}

	/**
	 * Finished when opened or closed
	 */
	@Override
	public boolean isFinished() {
		return _claw.getPosition() >= ManualClaw.CUBE_POS || _claw.getSwitchState();
	}
}