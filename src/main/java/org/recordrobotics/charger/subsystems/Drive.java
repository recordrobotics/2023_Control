// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger.subsystems;

import org.recordrobotics.charger.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@SuppressWarnings({"PMD.SystemPrintln", "PMD.FieldNamingConventions"})
public class Drive extends SubsystemBase {
	private WPI_TalonFX[] _left = {
		new WPI_TalonFX(RobotMap.DriveBase.LEFT_FRONT_MOTOR_PORT),
		new WPI_TalonFX(RobotMap.DriveBase.LEFT_MIDDLE_MOTOR_PORT),
		new WPI_TalonFX(RobotMap.DriveBase.LEFT_BACK_MOTOR_PORT)
	};
	private WPI_TalonFX[] _right = {
		new WPI_TalonFX(RobotMap.DriveBase.RIGHT_FRONT_MOTOR_PORT),
		new WPI_TalonFX(RobotMap.DriveBase.RIGHT_MIDDLE_MOTOR_PORT),
		new WPI_TalonFX(RobotMap.DriveBase.RIGHT_BACK_MOTOR_PORT)
	};

	private MotorControllerGroup _leftMotors = new MotorControllerGroup(_left);
	private MotorControllerGroup _rightMotors = new MotorControllerGroup(_right);

	private DifferentialDrive _differentialDrive = new DifferentialDrive(_leftMotors, _rightMotors);

	NavSensor gyro = new NavSensor();

	private static final FeedbackDevice SELECTED_SENSOR = FeedbackDevice.CTRE_MagEncoder_Relative;
	private TalonFXSensorCollection _leftCollection0 = new TalonFXSensorCollection(new BaseTalon(RobotMap.DriveBase.LEFT_FRONT_MOTOR_PORT, "origin"));
	private TalonFXSensorCollection _leftCollection1 = new TalonFXSensorCollection(new BaseTalon(RobotMap.DriveBase.LEFT_MIDDLE_MOTOR_PORT, "origin"));
	private TalonFXSensorCollection _leftCollection2 = new TalonFXSensorCollection(new BaseTalon(RobotMap.DriveBase.LEFT_BACK_MOTOR_PORT, "origin"));
	private TalonFXSensorCollection _rightCollection0 = new TalonFXSensorCollection(new BaseTalon(RobotMap.DriveBase.LEFT_FRONT_MOTOR_PORT, "origin"));
	private TalonFXSensorCollection _rightCollection1 = new TalonFXSensorCollection(new BaseTalon(RobotMap.DriveBase.LEFT_MIDDLE_MOTOR_PORT, "origin"));
	private TalonFXSensorCollection _rightCollection2 = new TalonFXSensorCollection(new BaseTalon(RobotMap.DriveBase.LEFT_BACK_MOTOR_PORT, "origin"));
		

	private static final double GEAR_RATIO = 10.75;
	private static final double WHEEL_DIAMETER = 6 * 25.4; // diameter in inches * conversion rate to millimeters

	//private final DifferentialDriveOdometry odometry;

	public Drive() {
		_leftMotors.set(0);
		_rightMotors.set(0);

		_left[0].configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 10);
		_left[1].configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 10);
		_left[2].configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 10);
		_right[0].configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 10);
		_right[1].configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 10);
		_right[2].configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 10);
	}

	/**
	 * drive the robot
	 * @param longSpeed forward/backward (positive is forward)
	 * @param latSpeed rotational speed (positive is clockwise)
	 */
	public void move(double longSpeed, double latSpeed) {
		// Arcade drive expects rotational inputs, while get translational
		// inputs. Therefore the values must be switched around
		// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html
		_differentialDrive.arcadeDrive(Subsystems.limitSpeed(latSpeed),
			Subsystems.limitSpeed(longSpeed));
	}

	/**
	 * converts encoder units
	 * @param position
	 * @return
	 */
	private double translateUnits(double position) {
		return position * (WHEEL_DIAMETER * Math.PI / GEAR_RATIO);
	}

	/**
	 * @return The value of the right encoder in MM
	 */
	public double getRightEncoder() {
		return (translateUnits(_rightCollection0.getIntegratedSensorPosition()) + translateUnits(_rightCollection1.getIntegratedSensorPosition()) + translateUnits(_rightCollection2.getIntegratedSensorPosition())) / 3;
	}

	/**
	 * @return The value of the left encoder in MM
	 */
	public double getLeftEncoder() {
		return (translateUnits(_leftCollection0.getIntegratedSensorPosition()) + translateUnits(_leftCollection1.getIntegratedSensorPosition()) + translateUnits(_leftCollection2.getIntegratedSensorPosition())) / 3;
	}

	/**
	 * @return The average value of the two encoders, left and right, in MM
	 */
	public double getPosition() {
		return (getRightEncoder() + getLeftEncoder()) / 2;
	}

	/**
	 * Reset all encoders to zero
	 */
	public void resetEncoders() {
		_leftCollection0.setIntegratedSensorPosition(0,0);
		_leftCollection1.setIntegratedSensorPosition(0,0);
		_leftCollection2.setIntegratedSensorPosition(0,0);
		_rightCollection0.setIntegratedSensorPosition(0,0);
		_rightCollection1.setIntegratedSensorPosition(0,0);
		_rightCollection2.setIntegratedSensorPosition(0,0);
	};

}
