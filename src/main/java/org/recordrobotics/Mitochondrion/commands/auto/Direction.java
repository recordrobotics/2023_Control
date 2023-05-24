package org.recordrobotics.Mitochondrion.commands.auto;

/**
 * Direction for auto commands
 */
public enum Direction {

	// Rotator & Drive
	BACKWARD(-1),
	FORWARD(1),
	// Climbers
	UP(-1),
	DOWN(1);

	private final int _value;

	private Direction(int value) {
		_value = value;
	}

	public int value() {
		return _value;
	}

}
