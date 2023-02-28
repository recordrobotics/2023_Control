package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class SingleControl implements IControlInput {

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
	public int getClawTurn() {
		if (_gamepad.getLeftBumperPressed()) {
			return -1;
		} else if (_gamepad.getRightBumperPressed()) {
			return 1;
		} else {
			return 0;
		}
	}

	@Override
	public String toString() {
		return "Single";
	}

}
