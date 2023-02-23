package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class SingleControl implements IControlInput {

	// private static final double TRIGGER_THRESHOLD = 0.25;

	private XboxController _gamepad;
	
	private double TRIGGER_THRESHOLD = 0.25;

	public SingleControl(int port) {
		_gamepad = new XboxController(port);
	}

	@Override
	public double getDriveLong() {
		return -_gamepad.getLeftY();
	}

	@Override
	public double getDriveLat() {
		return _gamepad.getLeftX();
	}

	@Override
	public ClawState getClawTurn() {
		if (_gamepad.getLeftTriggerAxis() < TRIGGER_THRESHOLD) {
			return ClawState.CLOSED;
		}
		else {
			return ClawState.NEUTRAL;
		}
	}

	@Override
	public String toString() {
		return "Single";
	}

}
