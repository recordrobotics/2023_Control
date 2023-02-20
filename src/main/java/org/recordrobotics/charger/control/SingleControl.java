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
	public boolean moveToSecond() {
		return _gamepad.getAButtonPressed();
	}

	@Override
	public boolean moveToThird() {
		return _gamepad.getBButtonPressed();
	}

	@Override
	public String toString() {
		return "Legacy";
	}

	@Override
	public boolean pickUpFromGround() {
		return _gamepad.getXButtonPressed();
	}

	@Override
	public boolean pickUpFromSub() {
		return _gamepad.getYButtonPressed();
	}

	//TODO: Finish This Code after writing the other thing
	@Override
	public ArmPosition getArmPosition(){
		return ArmPosition.DEFAULT;
	}
}
