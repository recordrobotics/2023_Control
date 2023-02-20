package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class SingleControl implements IControlInput {

	private static final double TRIGGER_THRESHOLD = 0.25;

	private XboxController _gamepad;

	private boolean _cube;
	private boolean _cone;
	private boolean _leftActivated;
	private boolean _rightActivated;

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
		if (_gamepad.getLeftTriggerAxis() < TRIGGER_THRESHOLD && !_leftActivated) {
			_cube = !_cube;
			_leftActivated = true;
		}
		if (_gamepad.getRightTriggerAxis() < TRIGGER_THRESHOLD && !_rightActivated) {
			_cone = !_cone;
			_rightActivated = true;
		}
		if (_gamepad.getLeftTriggerAxis() > TRIGGER_THRESHOLD && _leftActivated || _gamepad.getRightTriggerAxis() > TRIGGER_THRESHOLD && _rightActivated) {
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
		return "Single";
	}

}
