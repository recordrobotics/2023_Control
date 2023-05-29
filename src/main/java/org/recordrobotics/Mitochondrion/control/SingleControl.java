package org.recordrobotics.Mitochondrion.control;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SingleControl implements IControlInput {

	private XboxController _gamepad;

	private boolean _xLeftActivated = false;
	private boolean _xRightActivated = false;
	private boolean _yLeftActivated = false;
	private boolean _yRightActivated = false;

	private double _TRIGGER_THRESHOLD = 0.25;

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
	public ChangeAngle changeOriginAngle() {
		if (_gamepad.getRightX() > 0.5 && Math.abs(_gamepad.getRightY()) < 0.5 && !_xRightActivated) {
			_xRightActivated = true;
			return ChangeAngle.INCREASE;
		} else if (_gamepad.getRightX() < 0.5) {
			_xRightActivated = false;
		}
		if (_gamepad.getRightX() < -0.5 && Math.abs(_gamepad.getRightY()) < 0.5 && !_xLeftActivated) {
			_xLeftActivated = true;
			return ChangeAngle.DECREASE;
		} else if (_gamepad.getRightX() > -0.5) {
			_xLeftActivated = false;
		}
		return ChangeAngle.REMAIN;
	}

	@Override
	public ChangeAngle changeChangeAngle() {
		if (_gamepad.getRightY() > 0.5 && Math.abs(_gamepad.getRightX()) < 0.5 && !_yRightActivated) {
			_yRightActivated = true;
			return ChangeAngle.INCREASE;
		} else if (_gamepad.getRightY() < 0.5) {
			_yRightActivated = false;
		}
		if (_gamepad.getRightY() < -0.5 && Math.abs(_gamepad.getRightX()) < 0.5 && !_yLeftActivated) {
			_yLeftActivated = true;
			return ChangeAngle.DECREASE;
		} else if (_gamepad.getRightY() > -0.5) {
			_yLeftActivated = false;
		}
		return ChangeAngle.REMAIN;
	}

	@Override
	public ClawState getClawTurn() {
		if (_gamepad.getLeftTriggerAxis() >= _TRIGGER_THRESHOLD) {
			return ClawState.OPENING;
		} else if (_gamepad.getRightTriggerAxis() >= _TRIGGER_THRESHOLD) {
			return ClawState.GRABING;
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
	public int compArm() {
		if(_gamepad.getLeftBumper()) {
			return -1;
		} else if(_gamepad.getRightBumper()) {
			return 1;
		}
		return 0;
	}

	@Override
	public ArmPosition getArmPosition() {
		SmartDashboard.putBoolean("y button", _gamepad.getYButton());
		SmartDashboard.putBoolean("x button", _gamepad.getXButton());
		SmartDashboard.putBoolean("a button", _gamepad.getAButton());
		SmartDashboard.putBoolean("b button", _gamepad.getBButton());
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

	@Override
	public changeSetPointX changeSetPointX() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public changeSetPointY changeSetPointY() {
		// TODO Auto-generated method stub
		return null;
	}
}
