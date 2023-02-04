package org.recordrobotics.charger.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.recordrobotics.charger.subsystems.Claw;

public class AutoMoveClaw extends CommandBase{
	private boolean _endState;
	private Claw _claw;
	private double _speed;

	public AutoMoveClaw(Claw claw, double speed, boolean endState){
		if (speed <= 0) {
			throw new IllegalArgumentException("Speed must be positive");
		}
		if (claw == null) {
			throw new IllegalArgumentException("Claw is null");
		}

		_claw = claw;

		_speed = speed;

		_endState = endState;
	}

	/**
	 * Open/close claw
	 */
	@Override

	public void initialize() {
		_claw.resetEncoders();

		if(_claw.getPosition() > Claw.CLAW_NEUTRAL){
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
		if(_endState){
			return _claw.getPosition() >= Claw.CLAW_NEUTRAL || _claw.getPosition() <= Claw.CLAW_CUBE;
		} else{
			return _claw.getPosition() >= Claw.CLAW_NEUTRAL || _claw.getPosition() <= Claw.CLAW_CONE;
		}
	}
}
