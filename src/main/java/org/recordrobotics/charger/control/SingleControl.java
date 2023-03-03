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
		} else if (_gamepad.getRightBumperPressed()) {//this moves the arm back??????
			return 1;
		} else {
			return 0;
		}
	}

	@Override
	public String toString() {
		return "Single";
	}

	private int booleanToInt(boolean b) {
		return b ? 1 : 0;
	}

	@Override
	public ArmPosition getArmPosition() {
		boolean multiplePressed = (booleanToInt(_gamepad.getYButton())
			+ booleanToInt(_gamepad.getXButton())
			+ booleanToInt(_gamepad.getAButton())
			+ booleanToInt(_gamepad.getBButton()))
		> 1;
		if (multiplePressed) {
			return ArmPosition.NEUTRAL;
		}
		if (_gamepad.getYButton()) {
			return ArmPosition.THIRD;
		}
		if (_gamepad.getXButton()) {
			return ArmPosition.SECOND;
		}
		if (_gamepad.getBButton()) {
			return ArmPosition.SUBSTATION;
		}
		if (_gamepad.getAButton()) {
			return ArmPosition.GROUND;
		}
		return ArmPosition.NEUTRAL;
	}
}
