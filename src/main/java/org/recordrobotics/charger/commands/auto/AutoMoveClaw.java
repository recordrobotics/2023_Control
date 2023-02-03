package org.recordrobotics.charger.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.recordrobotics.charger.subsystems.Claw;

public class AutoMoveClaw extends CommandBase{
	private Claw _claw;
	private double _speed;

	public AutoMoveClaw(Claw claw, double speed){
		if (speed <= 0) {
			throw new IllegalArgumentException("Speed must be positive");
		}
		if (claw == null) {
			throw new IllegalArgumentException("Drive is null");
		}

		_claw = claw;

		_speed = speed;
	}

	/**
	 * Open/close claw
	 */
	@Override

	public void initialize() {
		_claw.resetEncoders();

		if(_claw.getPosition() < Claw.CLAW_NEUTRAL/2){
			_claw.turn(_speed);
		} else {
			_claw.turn(-_speed);
		}
	}

	/**
	 * Finished when opened or closed
	 */
	@Override
	public boolean isFinished() {
		return _claw.getPosition() >= Claw.CLAW_NEUTRAL || _claw.getPosition() <= 0;
	}
}
