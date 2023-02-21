package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class SingleControl implements IControlInput {

	//private static final double TRIGGER_THRESHOLD = 0.25;

	private XboxController _gamepad;

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
		// Trigger mimics button-like behavior
		boolean cube = _gamepad.getLeftBumper();
		boolean cone = _gamepad.getRightBumper();
		
		// Cube takes precedence
		if (cube) {
			return ClawState.CUBE;
		} else if (cone) {
			return ClawState.CONE;
		}
		return ClawState.NEUTRAL;
	}

	@Override
	public String toString() {
		return "Legacy";
	}

}
