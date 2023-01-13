package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class DoubleControl implements IControlInput {

	private static final double TRIGGER_THRESHOLD = 0.25;

	private XboxController _gamepad1;
	private XboxController _gamepad2;
	// Toggles for buttons on G2 - inversed when button is pressed
	private boolean _btnX;
	private boolean _btnY;

	public DoubleControl(int port1, int port2) {
		_gamepad1 = new XboxController(port1);
		_gamepad2 = new XboxController(port2);
	}

	@Override
	public double getDriveLong() {
		return -_gamepad1.getLeftY();
	}

	@Override
	public double getDriveLat() {
		return _gamepad1.getLeftX();
	}

	@Override
	public double getRotate() {
		return _gamepad2.getRightX();
	}

	@Override
	public double getClimb() {
		return _gamepad2.getLeftY();
	}

	@Override
	public double getAcqSpin() {
		// Forward mimics button-like behavior
		boolean forward = _gamepad1.getLeftTriggerAxis() > TRIGGER_THRESHOLD
				|| _gamepad1.getRightTriggerAxis() > TRIGGER_THRESHOLD;
		boolean backward = _gamepad1.getLeftBumper() || _gamepad1.getRightBumper();

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
		boolean out = _gamepad2.getLeftTriggerAxis() > TRIGGER_THRESHOLD
				|| _gamepad2.getRightTriggerAxis() > TRIGGER_THRESHOLD;
		boolean in = _gamepad2.getLeftBumper() || _gamepad2.getRightBumper();

		// Out takes precedence
		if (out)
			return 1;
		else if (in)
			return -1;
		return 0;
	}

	@Override
	public FlywheelState getFlywheel() {
		if (_gamepad2.getXButtonPressed())
			_btnX = !_btnX;
		// We still want to check Y, to reset it
		if (_gamepad2.getYButtonPressed())
			_btnY = !_btnY;

		// If X not pressed, clear Y
		if (!_btnX || _gamepad1.getXButtonPressed()) {
			_btnX = false;
			_btnY = false;
			return FlywheelState.OFF;
		}

		// Otherwise Y determines the state
		return _btnY ? FlywheelState.HIGH : FlywheelState.LOW;
	}

	@Override
	public boolean getServos() {
		return _gamepad1.getAButton() || _gamepad2.getAButton();
	}

	@Override
	public String toString() {
		return "Double";
	}

}
