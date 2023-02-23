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
	private TalonFXSensorCollection _changeCollection = new TalonFXSensorCollection(new BaseTalon(RobotMap.Arm.ORIGIN_MOTOR_PORT, "change"));
	private static final double ARM_LENGTH = 38;

	// IN DEGREES
	// negative is in the same direction as the rotation, positive is in the opposite
	private static final double ORIGIN_OFFSET = 0;

	private static final double TICKS_PER_REV = 2048;

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
			_changeMotor.set(speed);
		} else if (getChangeEncoder() < angles[1]) {
			_changeMotor.set(-speed);
		} else {
			_changeMotor.set(0);
		}
	}

	/**
	 * moves motors to reach a certain point on a cartesian plane, with the first motor as the origin point
	 * WARNING: DOES NOT WORK FOR GROUNDED PICKUPS
	 * @param targetX x value of the cartesian point
	 * @param targetY y value of the cartesian point
	 * @return angles of rotation in array of length 2 IN DEGREES
	 */
	public double[] getAnglesOfRotation(double targetX, double targetY) {
		double[] angles = new double[2];
		angles[0] = Math.toDegrees(Math.PI / 2 - Math.asin(targetY / ARM_LENGTH)) + ORIGIN_OFFSET;
		angles[1] = 180 - angles[0];
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
		return _originCollection.getIntegratedSensorPosition() / TICKS_PER_REV * 360;
	}

	/**
	 * @return value of change motor encoder in DEGREES
	 */
	public double getChangeEncoder() {
		return _changeCollection.getIntegratedSensorPosition() / TICKS_PER_REV * 360;
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
