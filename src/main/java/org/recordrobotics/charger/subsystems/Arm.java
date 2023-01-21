package org.recordrobotics.charger.subsystems;

import org.recordrobotics.charger.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	private WPI_TalonFX _originMotor = new WPI_TalonFX(RobotMap.Arm.ORIGIN_MOTOR_PORT);
	private WPI_TalonFX _changeMotor = new WPI_TalonFX(RobotMap.Arm.CHANGE_MOTOR_PORT);

	private static final FeedbackDevice SELECTED_SENSOR = FeedbackDevice.CTRE_MagEncoder_Relative;

	private static final double FIRST_ARM_LENGTH = 0;
	private static final double SECOND_ARM_LENGTH = 0;

	public Arm() {
		_originMotor.configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 10);
		_changeMotor.configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 10);
	}

	/**
	 * moves motors to reach a certain point on a cartesian plane, with the first motor as the origin point
	 * @param targetX x value of the cartesian point
	 * @param targetY y value of the cartesian point
	 * @return angles of rotation in array of length 2 IN DEGREES
	 */
	public double[] getAnglesOfRotation(double targetX, double targetY) {
		// law of cosines
		double side3 = Math.sqrt(Math.pow(targetX, 2) + Math.pow(targetY, 2));
		double angleC = Math.toDegrees(Math.acos((Math.pow(SECOND_ARM_LENGTH, 2) - Math.pow(FIRST_ARM_LENGTH, 2) - Math.pow(side3, 2))/(side3 * FIRST_ARM_LENGTH * 2)));
		// angle of rotation for the first motor
		double angle1 = (90 - angleC - Math.toDegrees(Math.tan(targetY / targetX)));
		// law of sines
		// angle of rotation for the second motor
		double angle2 = Math.toDegrees(Math.asin(side3 * Math.sin(angleC) / SECOND_ARM_LENGTH));
		return new double[] {angle1, angle2};
	}
}
