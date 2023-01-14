package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class LegacyControl implements IControlInput {

	// private static final double TRIGGER_THRESHOLD = 0.25;

	private XboxController _gamepad;

	public LegacyControl(int port) {
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
	public String toString() {
		return "Legacy";
	}

}
