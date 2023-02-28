package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class DoubleControl implements IControlInput {

	private XboxController _gamepad1;
	private XboxController _gamepad2;

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
	public int getClawTurn() {
		if (_gamepad2.getLeftBumperPressed()) {
			return -1;
		} else if (_gamepad2.getRightBumperPressed()) {
			return 1;
		} else {
			return 0;	
		}
	}

	@Override
	public String toString() {
		return "Double";
	}

}
