package org.recordrobotics.charger.subsystems;

import org.recordrobotics.charger.Constants;
import org.recordrobotics.charger.RobotMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	private WPI_TalonFX _originMotor = new WPI_TalonFX(RobotMap.Arm.ORIGIN_MOTOR_PORT);
	private WPI_TalonFX _changeMotor = new WPI_TalonFX(RobotMap.Arm.CHANGE_MOTOR_PORT);
	private TalonFXSensorCollection _originCollection = new TalonFXSensorCollection(new BaseTalon(RobotMap.Arm.ORIGIN_MOTOR_PORT, "origin"));
	private TalonFXSensorCollection _changeCollection = new TalonFXSensorCollection(new BaseTalon(RobotMap.Arm.CHANGE_MOTOR_PORT, "change"));
	private static final double FIRST_ARM_LENGTH = 30;
	private static final double SECOND_ARM_LENGTH = 30;

	// IN DEGREES
	// negative is in the same direction as the rotation, positive is in the opposite
	private static final double ORIGIN_OFFSET = 0;
	private static final double CHANGE_OFFSET = 0;

	private static final double TICKS_PER_REV = 2048;
	private static final double GEAR_RATIO = 16;

	private GenericEntry _entryAngles;

	private double[] _angles = new double[2];

	public Arm() {
		_originCollection.setIntegratedSensorPosition(0, 0);
		_changeCollection.setIntegratedSensorPosition(0, 0);
		_originMotor.setNeutralMode(NeutralMode.Brake);
		_changeMotor.setNeutralMode(NeutralMode.Brake);

		ShuffleboardTab tab = Shuffleboard.getTab(Constants.DATA_TAB);
		_entryAngles = tab.add("Angles Of Rotation", new double[] {0, 0}).getEntry();
	}

	/**
	 * Moves motors to set angles (pid?)
	 * @param angles the angles to turn the motors (first = origin motor, second = change motor)
	 */
	public void moveAngles(double speed, double... angles) {
		if (getOriginEncoder() < angles[0]) {
			_originMotor.set(speed);
		} else if (getOriginEncoder() > angles[0]) {
			_originMotor.set(-speed);
		} else {
			_originMotor.set(0);
		}
		if (getChangeEncoder() > angles[1]) {
			_changeMotor.set(-speed);
		} else if (getChangeEncoder() < angles[1]) {
			_changeMotor.set(speed);
		} else {
			_changeMotor.set(0);
		}
	}

	public void spinOrigin(double speed){
		_originMotor.set(speed);
	}

	public void spinChange(double speed){
		_changeMotor.set(speed);
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
		angles[0] = -(Math.toDegrees(2 * Math.PI - angleC - Math.atan(targetY / targetX)) + ORIGIN_OFFSET);
		// law of sines
		// angle of rotation for the second motor
		angles[1] = -(Math.toDegrees(Math.asin(side3 * Math.sin(angleC) / SECOND_ARM_LENGTH)) + CHANGE_OFFSET);
		_angles = angles;
		return angles;
	}

	/**
	 * moves motors to reach a certain point on a cartesian plane, with the first motor as the origin point
	 * keeps second arm parallel to the ground
	 * @param targetY y value of the cartesian point
	 * @return angles of rotation in array of length 2 IN DEGREES
	 */
	public double[] getRelatedAngles(double targetY) {
		double[] angles = new double[2];
		// core angle is the complement of angle 1 and supplement of angle 2
		double coreAngle = Math.asin(targetY / FIRST_ARM_LENGTH);
		angles[0] = Math.toDegrees(Math.PI / 2 - coreAngle) + ORIGIN_OFFSET;
		angles[1] = Math.toDegrees(Math.PI - coreAngle) + ORIGIN_OFFSET;
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
		return _originMotor.getSelectedSensorPosition() / TICKS_PER_REV * 360 / GEAR_RATIO;
	}

	/**
	 * @return value of change motor encoder in DEGREES
	 */
	public double getChangeEncoder() {
		return _changeMotor.getSelectedSensorPosition() / TICKS_PER_REV * 360 / GEAR_RATIO;
	}

	/**
	 * resets origin and change motor encoders
	 */
	public void resetEncoders() {
		_originCollection.setIntegratedSensorPosition(0, 0);
		_changeCollection.setIntegratedSensorPosition(0, 0);
	}

	@Override
	public void periodic() {
		_entryAngles.setDoubleArray(_angles);
	}
}
