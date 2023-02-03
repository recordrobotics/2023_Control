package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class SingleControl implements IControlInput {

	private static final double TRIGGER_THRESHOLD = 0.25;

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
	public double getClawTurn() {
		// Out mimics button-like behavior
		boolean cube = _gamepad.getLeftTriggerAxis() > TRIGGER_THRESHOLD;
		boolean cone = _gamepad.getRightTriggerAxis() > TRIGGER_THRESHOLD;

		// Out takes precedence
		if (cube)
			return 1;
		else if (cone)
			return -1;
		return 0;
	}

	@Override
	public String toString() {
		return "Legacy";
	}

}
