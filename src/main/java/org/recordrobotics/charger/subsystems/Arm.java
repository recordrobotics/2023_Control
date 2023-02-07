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

	private static final double FIRST_ARM_LENGTH = 0;
	private static final double SECOND_ARM_LENGTH = 0;

	// IN DEGREES
	// if the motor arms are in the first quadrant on a cartesian plane, these angles are in the second quadrant.
	private static final double ORIGIN_ANGLE_FROM_VERTICAL = 0;
	private static final double CHANGE_ANGLE_FROM_VERTICAL = 0;

	private GenericEntry _entryAngles;

	private double[] _angles = new double[2];

	public Arm() {
		_originMotor.configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 10);
		_changeMotor.configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 10);

		ShuffleboardTab tab = Shuffleboard.getTab("data");
		_entryAngles = tab.add("Angles Of Rotation", new double[] {0, 0}).getEntry();
	}

	public void moveAngles(double[] angles) {
		// move angles
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
		angles[0] = Math.toDegrees(2 * Math.PI - angleC - Math.atan(targetY / targetX)) + ORIGIN_ANGLE_FROM_VERTICAL;
		// law of sines
		// angle of rotation for the second motor
		angles[1] = Math.toDegrees(Math.asin(side3 * Math.sin(angleC) / SECOND_ARM_LENGTH)) + CHANGE_ANGLE_FROM_VERTICAL;
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

	@Override
	public void periodic() {
		_entryAngles.setDoubleArray(_angles);
	}
}
