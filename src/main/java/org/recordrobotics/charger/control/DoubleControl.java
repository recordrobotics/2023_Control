package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class DoubleControl implements IControlInput {

	// private static final double TRIGGER_THRESHOLD = 0.25;

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
		return -_gamepad1.getLeftX();
	}

	@Override
	public String toString() {
		return "Double";
	}

	private int booleanToInt(boolean b) {
		return b ? 1 : 0;
	}

	@Override
	public ArmPosition getArmPosition() {
		boolean multiplePressed = booleanToInt(_gamepad2.getYButtonPressed()) + booleanToInt(_gamepad2.getXButtonPressed()) + booleanToInt(_gamepad2.getAButtonPressed()) + booleanToInt(_gamepad2.getBButtonPressed()) > 1;
		if (multiplePressed) {
			return ArmPosition.DEFAULT;
		} else {
			if (_gamepad2.getYButtonPressed()) {
				return ArmPosition.THIRD;
			} if (_gamepad2.getXButtonPressed()) {
				return ArmPosition.SECOND;
			} if (_gamepad2.getBButtonPressed()) {
				return ArmPosition.SUBSTATION;
			} if (_gamepad2.getAButtonPressed()) {
				return ArmPosition.GROUND;
			} else {
				return ArmPosition.DEFAULT;
			}
		}
	}
}
