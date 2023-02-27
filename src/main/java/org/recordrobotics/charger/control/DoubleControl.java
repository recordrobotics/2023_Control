package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class DoubleControl implements IControlInput {

	private XboxController _gamepad1;
	private XboxController _gamepad2;

	private boolean _cube;
	private boolean _cone;

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
		if (_gamepad2.getLeftBumperPressed()) {
			_cube = !_cube;
		}
		if (_gamepad2.getRightBumperPressed()) {
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
		return "Double";
	}

}
