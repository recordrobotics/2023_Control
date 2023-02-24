// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger.subsystems;

import org.recordrobotics.charger.Constants;
import org.recordrobotics.charger.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

	private static final FeedbackDevice SELECTED_SENSOR = FeedbackDevice.IntegratedSensor;

	private static final double GEAR_RATIO = 10.75;
	private static final double WHEEL_DIAMETER = 6 * 25.4; // diameter in inches * conversion rate to millimeters

	private GenericEntry _encoderEntry;

	public Drive() {
		_leftMotors.set(0);
		_rightMotors.set(0);
		_left[0].configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 0);
		_left[1].configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 0);
		_left[2].configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 0);
		_right[0].configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 0);
		_right[1].configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 0);
		_right[2].configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 0);
		_left[0].setSelectedSensorPosition(0);
		_left[1].setSelectedSensorPosition(0);
		_left[2].setSelectedSensorPosition(0);
		_right[0].setSelectedSensorPosition(0);
		_right[1].setSelectedSensorPosition(0);
		_right[2].setSelectedSensorPosition(0);

		ShuffleboardTab tab = Shuffleboard.getTab(Constants.DATA_TAB);
		_encoderEntry = tab.add("Drive Encoders", 0).getEntry();
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
		return position / (WHEEL_DIAMETER * Math.PI / GEAR_RATIO) * 2;
	}

	/**
	 * @return The value of the right encoder in MM
	 */
	private double getRightEncoder() {
		return (translateUnits(_right[0].getSelectedSensorPosition())
			+ translateUnits(_right[1].getSelectedSensorPosition())
			+ translateUnits(_right[2].getSelectedSensorPosition()))
		/ 3;
	}

	/**
	 * @return The value of the left encoder in MM
	 */
	private double getLeftEncoder() {
		return -(translateUnits(_left[0].getSelectedSensorPosition())
			+ translateUnits(_left[1].getSelectedSensorPosition())
			+ translateUnits(_left[2].getSelectedSensorPosition()))
		/ 3;
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
		_left[0].setSelectedSensorPosition(0.0);
		_left[1].setSelectedSensorPosition(0.0);
		_left[2].setSelectedSensorPosition(0.0);
		_right[0].setSelectedSensorPosition(0.0);
		_right[1].setSelectedSensorPosition(0.0);
		_right[2].setSelectedSensorPosition(0.0);
	};

	@Override
	public void periodic() {
		_encoderEntry.setDouble(translateUnits(getPosition()));
	}
}
