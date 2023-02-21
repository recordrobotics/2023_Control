package org.recordrobotics.charger.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;

import org.recordrobotics.charger.commands.manual.ManualClaw;
import org.recordrobotics.charger.subsystems.Claw;

public class AutoMoveClaw extends CommandBase{
	private Claw _claw;
	private double _speed;
	private boolean _endState;

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

		if(_claw.getPosition() < ManualClaw.CONE_POS/2){
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
		if (_endState) {
			return _claw.getPosition() >= ManualClaw.CONE_POS|| _claw.getPosition() <= 0;
		} 
		return _claw.getPosition() >= ManualClaw.CUBE_POS|| _claw.getPosition() <= 0;
	}
}
