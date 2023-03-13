package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class SingleControl implements IControlInput {

	private XboxController _gamepad;

	private double TRIGGER_THRESHOLD = 0.25;

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
		if (_gamepad.getLeftTriggerAxis() > TRIGGER_THRESHOLD) {
			return ClawState.GRAB;
		}
		if (_gamepad.getRightTriggerAxis() > TRIGGER_THRESHOLD) {
			return ClawState.RELEASE;
		}	
		return ClawState.NEUTRAL;
	}

	@Override
	public String toString() {
		return "Single";
	}

	@Override
	public SpeedState speedState() {
		if (_gamepad.getLeftBumper()) {
			return SpeedState.SLOW;
		}
		return SpeedState.NEUTRAL;
	}

	@Override
	public boolean canTurn() {
		return !_gamepad.getRightBumper();
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
