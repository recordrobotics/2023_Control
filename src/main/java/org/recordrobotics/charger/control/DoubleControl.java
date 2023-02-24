package org.recordrobotics.charger.control;

import edu.wpi.first.wpilibj.XboxController;

public class DoubleControl implements IControlInput {

	// private static final double TRIGGER_THRESHOLD = 0.25;

	private XboxController _gamepad1;
	@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
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
		return _gamepad1.getLeftX();
	}

	@Override
	public String toString() {
		return "Double";
	}
}
