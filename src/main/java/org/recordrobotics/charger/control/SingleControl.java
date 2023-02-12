package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class SingleControl implements IControlInput {

	private static final double TRIGGER_THRESHOLD = 0.25;

	private XboxController _gamepad;

	private boolean _lTrig;
	private boolean _rTrig;

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
		// Out mimics button-like behavior
		boolean cube = _gamepad.getLeftTriggerAxis() > TRIGGER_THRESHOLD;
		boolean cone = _gamepad.getRightTriggerAxis() > TRIGGER_THRESHOLD;

		if (_gamepad.getLeftTriggerAxis() > TRIGGER_THRESHOLD)
			_lTrig = !_lTrig;

		if (_gamepad.getRightTriggerAxis() > TRIGGER_THRESHOLD)
			_rTrig = !_rTrig;

		// Cube takes precedence
		if (cube || cone){
			return _lTrig ? ClawState.CUBE : ClawState.CONE;
		}
		return ClawState.NEUTRAL;
	}

	@Override
	public String toString() {
		return "Legacy";
	}

}
