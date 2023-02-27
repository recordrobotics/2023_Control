package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class SingleControl implements IControlInput {

	private XboxController _gamepad;

	private boolean _cube;
	private boolean _cone;

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
		if (_gamepad.getLeftBumperPressed()) {
			_cube = !_cube;
		}
		if (_gamepad.getRightBumperPressed()) {
			_cone = !_cone;
		}

		if (_cube) {
			return ClawState.CUBE;
		} else if (_cone) {
			return ClawState.CONE;
		} else {
			return ClawState.NEUTRAL;
		}
	}

	@Override
	public String toString() {
		return "Single";
	}

}
