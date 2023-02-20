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
	public boolean moveToSecond() {
		return _gamepad2.getAButtonPressed();
	}

	@Override
	public boolean moveToThird() {
		return _gamepad2.getBButtonPressed();
	}

	@Override
	public String toString() {
		return "Double";
	}

	@Override
	public boolean pickUpFromGround() {
		return _gamepad2.getYButtonPressed();
	}

	@Override
	public boolean pickUpFromSub() {
		return _gamepad2.getYButtonPressed();
	}

	private int booleanToInt(boolean b){
		if(b == true){
			return 1;
		}else{
			return 0;
		}
	}

	@Override
	public ArmPosition getArmPosition(){
		if(booleanToInt(_gamepad2.getYButtonPressed()) + booleanToInt(_gamepad2.getXButtonPressed()) + booleanToInt(_gamepad2.getAButtonPressed()) + booleanToInt(_gamepad2.getBButtonPressed())> 1){
			return ArmPosition.DEFAULT;
		}else{
			if(_gamepad2.getYButtonPressed()){
				return ArmPosition.THIRD;
			}
			if(_gamepad2.getXButtonPressed()){
				return ArmPosition.SECOND;
			}
			if(_gamepad2.getBButtonPressed()){
				return ArmPosition.SUBSTATION;
			}
			if(_gamepad2.getAButtonPressed()){
				return ArmPosition.GROUND;
			}else{
				return ArmPosition.DEFAULT;
			}
		}
	}
}
