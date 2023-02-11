package org.recordrobotics.charger.subsystems;

import org.recordrobotics.charger.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	private WPI_TalonFX _originMotor = new WPI_TalonFX(RobotMap.Arm.ORIGIN_MOTOR_PORT);
	private WPI_TalonFX _changeMotor = new WPI_TalonFX(RobotMap.Arm.CHANGE_MOTOR_PORT);

	private static final FeedbackDevice SELECTED_SENSOR = FeedbackDevice.CTRE_MagEncoder_Relative;

	private static final double FIRST_ARM_LENGTH = 30;
	private static final double SECOND_ARM_LENGTH = 30;

	// IN DEGREES
	// negative is in the same direction as the rotation, positive is in the opposite
	private static final double ORIGIN_OFFSET = 0;
	private static final double CHANGE_OFFSET = 0;

	private static final double TICKS_PER_REV = 2048;

	private GenericEntry _entryAngles;

	private double[] _angles = new double[2];

	public Arm() {
		_originMotor.configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 10);
		_changeMotor.configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 10);

		ShuffleboardTab tab = Shuffleboard.getTab("data");
		_entryAngles = tab.add("Angles Of Rotation", new double[] {0, 0}).getEntry();
	}

	/**
	 * Moves motors to set angles (pid?)
	 * @param angles the angles to turn the motors (first = origin motor, second = change motor)
	 */
	public void moveAngles(double[] angles) {
		_originMotor.set(0.5);
		while (true) {
			if (getOriginEncoder() >= angles[0]) {
				_originMotor.set(0);
				break;
			}
		}
		_changeMotor.set(0.5);
		while (true) {
			if (getChangeEncoder() >= angles[0]) {
				_changeMotor.set(0);
				break;
			}
		}
	}

	/**
	 * moves motors to reach a certain point on a cartesian plane, with the first motor as the origin point
	 * @param targetX x value of the cartesian point
	 * @param targetY y value of the cartesian point
	 * @return angles of rotation in array of length 2 IN DEGREES
	 */
	public double[] getAnglesOfRotation(double targetX, double targetY) {
		double[] angles = new double[2];
		// law of cosines
		double side3 = Math.sqrt(Math.pow(targetX, 2) + Math.pow(targetY, 2));
		double angleC = Math.acos((Math.pow(SECOND_ARM_LENGTH, 2) - Math.pow(FIRST_ARM_LENGTH, 2) - Math.pow(side3, 2))/(side3 * FIRST_ARM_LENGTH * 2));
		// angle of rotation for the first motor
		angles[0] = Math.toDegrees(2 * Math.PI - angleC - Math.atan(targetY / targetX)) + ORIGIN_OFFSET;
		// law of sines
		// angle of rotation for the second motor
		angles[1] = Math.toDegrees(Math.asin(side3 * Math.sin(angleC) / SECOND_ARM_LENGTH)) + CHANGE_OFFSET;
		_angles = angles;
		return angles;
	}

	/**
	 * resets the position of the arms
	 * @return the original angles of the arms
	 */
	public double[] resetPositions() {
		return new double[] {0, 0};
	}

	/**
	 * @return value of origin motor encoder in DEGREES
	 */
	public double getOriginEncoder() {
		return _originMotor.getSelectedSensorPosition() / TICKS_PER_REV * 360;
	}

	/**
	 * @return value of change motor encoder in DEGREES
	 */
	public double getChangeEncoder() {
		return _changeMotor.getSelectedSensorPosition() / TICKS_PER_REV * 360;
	}

	/**
	 * resets origin and change motor encoders
	 */
	public void resetEncoders() {
		_originMotor.setSelectedSensorPosition(0);
		_changeMotor.setSelectedSensorPosition(0);
	}

	@Override
	public void periodic() {
		_entryAngles.setDoubleArray(_angles);
	}
}
