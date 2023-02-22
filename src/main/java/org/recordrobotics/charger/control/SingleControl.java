package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class SingleControl implements IControlInput {

	// private static final double TRIGGER_THRESHOLD = 0.25;

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
	public String toString() {
		return "Legacy";
	}

	private int booleanToInt(boolean b) {
		return b ? 1 : 0;
	}

	@Override
	public ArmPosition getArmPosition() {
		boolean multiplePressed = booleanToInt(_gamepad.getYButtonPressed())
			+ booleanToInt(_gamepad.getXButtonPressed())
			+ booleanToInt(_gamepad.getAButtonPressed())
			+ booleanToInt(_gamepad.getBButtonPressed())
		> 1;
		if (multiplePressed) {
			return ArmPosition.NEUTRAL;
		}
		if (_gamepad.getYButtonPressed()) {
			return ArmPosition.THIRD;
		}
		if (_gamepad.getXButtonPressed()) {
			return ArmPosition.SECOND;
		}
		if (_gamepad.getBButtonPressed()) {
			return ArmPosition.SUBSTATION;
		}
		if (_gamepad.getAButtonPressed()) {
			return ArmPosition.GROUND;
		}
		return ArmPosition.NEUTRAL;
	}
}
