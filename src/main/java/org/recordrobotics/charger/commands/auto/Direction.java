package org.recordrobotics.charger.commands.auto;

/**
 * Direction for auto commands
 */
public enum Direction {

	// Drive
	BACKWARD(-1),
	FORWARD(1);

	private final int _value;

	private Direction(int value) {
		_value = value;
	}

	public int value() {
		return _value;
	}

}
