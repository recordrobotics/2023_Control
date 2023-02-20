package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class DoubleControl implements IControlInput {

	private static final double TRIGGER_THRESHOLD = 0.25;

	private XboxController _gamepad1;
	private XboxController _gamepad2;

	private boolean _cube;
	private boolean _cone;
	private boolean _leftActivated;
	private boolean _rightActivated;

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
		return -_gamepad2.getLeftX();
	}

	@Override
	public ClawState getClawTurn() {
		if (_gamepad2.getLeftTriggerAxis() < TRIGGER_THRESHOLD && !_leftActivated) {
			_cube = !_cube;
			_leftActivated = true;
		}
		if (_gamepad2.getRightTriggerAxis() < TRIGGER_THRESHOLD && !_rightActivated) {
			_cone = !_cone;
			_rightActivated = true;
		}
		if (_gamepad2.getLeftTriggerAxis() > TRIGGER_THRESHOLD && _leftActivated || _gamepad2.getRightTriggerAxis() > TRIGGER_THRESHOLD && _rightActivated) {
			_leftActivated = false;
			_rightActivated = false;
		}
		if (!_cube && !_cone) {
			return ClawState.NEUTRAL;
		} else if (_cube) {
			return ClawState.CUBE;
		} else {
			return ClawState.CONE;
		}
	}

	@Override
	public String toString() {
		return "Double";
	}

}
