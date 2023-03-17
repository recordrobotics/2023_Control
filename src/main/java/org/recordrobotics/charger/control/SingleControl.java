package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class SingleControl implements IControlInput {

<<<<<<< HEAD
	// private static final double TRIGGER_THRESHOLD = 0.25;
=======
	//private static final double TRIGGER_THRESHOLD = 0.25;
>>>>>>> 076f860ed7bc098ed1ce03a391e0afb85b5722bd

	private XboxController _gamepad;

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
	public ClawState getClawTurn() {
<<<<<<< HEAD
		if (_gamepad.getLeftTriggerAxis() < _TRIGGER_THRESHOLD) {
=======
		// Trigger mimics button-like behavior
		boolean cube = _gamepad.getLeftBumper();
		boolean cone = _gamepad.getRightBumper();
		
		// Cube takes precedence
		if (cube) {
>>>>>>> 076f860ed7bc098ed1ce03a391e0afb85b5722bd
			return ClawState.CUBE;
		}
		else if (_gamepad.getRightTriggerAxis() < _TRIGGER_THRESHOLD) {
			return ClawState.CONE;
		}
<<<<<<< HEAD
		else {
			return ClawState.NEUTRAL;
		}
=======
		return ClawState.NEUTRAL;
>>>>>>> 076f860ed7bc098ed1ce03a391e0afb85b5722bd
	}

	@Override
	public String toString() {
		return "Legacy";
	}

}
