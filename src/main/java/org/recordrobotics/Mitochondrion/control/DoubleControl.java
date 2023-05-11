package org.recordrobotics.Mitocondrion.control;

import edu.wpi.first.wpilibj.XboxController;

public class DoubleControl implements IControlInput {

	private XboxController _gamepad1;
	private XboxController _gamepad2;

	private boolean _xLeftActivated = false;
	private boolean _xRightActivated = false;
	private boolean _yLeftActivated = false;
	private boolean _yRightActivated = false;

	private double _TRIGGER_THRESHOLD = 0.75;

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
	public ChangeAngle changeOriginAngle() {
		if (_gamepad2.getRightX() > 0.5 && Math.abs(_gamepad2.getRightY()) < 0.5 && !_xRightActivated) {
			_xRightActivated = true;
			return ChangeAngle.INCREASE;
		} else if (_gamepad2.getRightX() < 0.5) {
			_xRightActivated = false;
		}
		if (_gamepad2.getRightX() < -0.5 && Math.abs(_gamepad2.getRightY()) < 0.5 && !_xLeftActivated) {
			_xLeftActivated = true;
			return ChangeAngle.DECREASE;
		} else if (_gamepad2.getRightX() > -0.5) {
			_xLeftActivated = false;
		}
		return ChangeAngle.REMAIN;
	}

	@Override
	public ChangeAngle changeChangeAngle() {
		if (_gamepad2.getRightY() > 0.5 && Math.abs(_gamepad2.getRightX()) < 0.5 && !_yRightActivated) {
			_yRightActivated = true;
			return ChangeAngle.INCREASE;
		} else if (_gamepad2.getRightY() < 0.5) {
			_yRightActivated = false;
		}
		if (_gamepad2.getRightY() < -0.5 && Math.abs(_gamepad2.getRightX()) < 0.5 && !_yLeftActivated) {
			_yLeftActivated = true;
			return ChangeAngle.DECREASE;
		} else if (_gamepad2.getRightY() > -0.5) {
			_yLeftActivated = false;
		}
		return ChangeAngle.REMAIN;
	}

	@Override
	public ClawState getClawTurn() {
		if (_gamepad2.getLeftTriggerAxis() >= _TRIGGER_THRESHOLD) {
			return ClawState.OPENING;
		} else if (_gamepad2.getRightTriggerAxis() >= _TRIGGER_THRESHOLD) {
			return ClawState.GRABING;
		}
		return ClawState.NEUTRAL;
	}

	@Override
	public String toString() {
		return "Double";
	}

	@Override
	public SpeedState speedState() {
		if (_gamepad1.getBButton()) {
			return SpeedState.SLOW;
		}
		if (_gamepad1.getLeftBumper()) {
			return SpeedState.FAST;
		}
		return SpeedState.NEUTRAL;
	}

	@Override
	public int compArm() {
		if(_gamepad2.getLeftBumper()) {
			return -1;
		} else if(_gamepad2.getRightBumper()) {
			return 1;
		}
		return 0;
	}

	@Override
	public boolean canTurn() {
		return !_gamepad1.getRightBumper();
}

	private int booleanToInt(boolean b) {
		return b ? 1 : 0;
	}

	@Override
	public ArmPosition getArmPosition() {
		boolean multiplePressed = (booleanToInt(_gamepad2.getYButton())
			+ booleanToInt(_gamepad2.getXButton())
			+ booleanToInt(_gamepad2.getAButton())
			+ booleanToInt(_gamepad2.getBButton()))
		> 1;
		if (multiplePressed) {
			return ArmPosition.NEUTRAL;
		}
		if (_gamepad2.getYButton()) {
			return ArmPosition.THIRD;
		}
		if (_gamepad2.getXButton()) {
			return ArmPosition.SECOND;
		}
		if (_gamepad2.getBButton()) {
			return ArmPosition.SUBSTATION;
		}
		if (_gamepad2.getAButton()) {
			return ArmPosition.GROUND;
		}
		return ArmPosition.NEUTRAL;
	}
}
