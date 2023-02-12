package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class DoubleControl implements IControlInput {

	private static final double TRIGGER_THRESHOLD = 0.25;

	private XboxController _gamepad1;
	private XboxController _gamepad2;

	private boolean _lTrig;
	private boolean _rTrig;

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
	public ClawState getClawTurn() {
		// Out mimics button-like behavior
		boolean cube = _gamepad2.getLeftTriggerAxis() > TRIGGER_THRESHOLD;
		boolean cone = _gamepad2.getRightTriggerAxis() > TRIGGER_THRESHOLD;

		if (_gamepad2.getLeftTriggerAxis() > TRIGGER_THRESHOLD)
			_lTrig = !_lTrig;

		if (_gamepad2.getRightTriggerAxis() > TRIGGER_THRESHOLD)
			_rTrig = !_rTrig;

		// Cube takes precedence
		if (cube || cone){
			return _lTrig ? ClawState.CUBE : ClawState.CONE;
		}
		return ClawState.NEUTRAL;
	}

	@Override
	public String toString() {
		return "Double";
	}

}
