package org.recordrobotics.charger.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.recordrobotics.charger.subsystems.Claw;

public class AutoMoveClaw extends CommandBase{
	private Claw _claw;
	private double _speed;
	private int _status;

	private int _grab = 1;
	private int _release = -1;

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
		if (_status == _grab) {
			if (_claw.getCurrent() > _claw._CURRENT_GRAB_THRESHOLD) {
				_claw.turn(0);
			}
			else {
				_claw.turn(-_speed);
			}
		} else if (_status == _release) {
			if (_claw.getSwitchState()) {
				_claw.turn(_speed);
			}
			else {
				_claw.turn(0);
			}
		}
	}

	/**
	 * Finished when opened or closed
	 */
	@Override
	public boolean isFinished() {
		return _claw.getSwitchState() || _claw.getCurrent() > _claw._CURRENT_GRAB_THRESHOLD;
	}

	public void end(boolean interrupted) {
		_claw.turn(0);
	}
}