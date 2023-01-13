package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class LegacyControl implements IControlInput {

	private static final double TRIGGER_THRESHOLD = 0.25;

	private XboxController _gamepad;
	// Toggles for buttons - inversed when button is pressed
	private boolean _btnX;
	private boolean _btnY;

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
	public double getRotate() {
		return _gamepad.getRightX();
	}

	@Override
	public double getClimb() {
		return _gamepad.getRightY();
	}

	@Override
	public double getAcqSpin() {
		// Forward mimics button-like behavior
		boolean forward = _gamepad.getLeftTriggerAxis() > TRIGGER_THRESHOLD;
		boolean backward = _gamepad.getLeftBumper();

		// Forward takes precedence
		if (forward)
			return 1;
		else if (backward)
			return -1;
		return 0;
	}

	@Override
	public double getAcqTilt() {
		// Out mimics button-like behavior
		boolean out = _gamepad.getRightTriggerAxis() > TRIGGER_THRESHOLD;
		boolean in = _gamepad.getRightBumper();

		// Out takes precedence
		if (out)
			return 1;
		else if (in)
			return -1;
		return 0;
	}

	@Override
	public FlywheelState getFlywheel() {
		if (_gamepad.getXButtonPressed())
			_btnX = !_btnX;
		// We still want to check Y, to reset it
		if (_gamepad.getYButtonPressed())
			_btnY = !_btnY;

		// If X not pressed, clear Y
		if (!_btnX) {
			_btnY = false;
			return FlywheelState.OFF;
		}

		// Otherwise Y determines the state
		return _btnY ? FlywheelState.HIGH : FlywheelState.LOW;
	}

	@Override
	public boolean getServos() {
		return _gamepad.getAButton();
	}

	@Override
	public String toString() {
		return "Legacy";
	}

}
