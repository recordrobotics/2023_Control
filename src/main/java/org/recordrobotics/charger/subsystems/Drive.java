// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger.subsystems;

import java.util.function.Consumer;

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
	// Diameter in inches * conversion rate to millimeters
	private static final double WHEEL_DIAMETER = 6 * 25.4;
	// Units per millimeter
	private static final double UPMM = WHEEL_DIAMETER * Math.PI * 2 / GEAR_RATIO;

	private GenericEntry _encoderEntry;

	public Drive() {
		runForAllMotors(m -> {
			m.configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 0);
			m.setSelectedSensorPosition(0);
			m.set(0);
		});

		ShuffleboardTab tab = Shuffleboard.getTab(Constants.DATA_TAB);
		_encoderEntry = tab.add("Drive Encoders", 0).getEntry();
	}

	@Override
	public void periodic() {
		_encoderEntry.setDouble(getPosition());
	}

	/**
	 * Drive the robot
	 * @param longSpeed forward/backward (positive is forward)
	 * @param latSpeed rotational speed (positive is clockwise)
	 */
	public void move(double longSpeed, double latSpeed) {
		// Arcade drive expects rotational inputs, while get translational
		// inputs. Therefore the values must be switched around
		// https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html
		_differentialDrive.arcadeDrive(Subsystems.limitSpeed(-latSpeed),
			Subsystems.limitSpeed(-longSpeed));
	}

	/**
	 * @return The value of the right encoder in MM
	 */
	public double getRightEncoder() {
		return averageEncoders(_right);
	}

	/**
	 * @return The value of the left encoder in MM
	 */
	public double getLeftEncoder() {
		return averageEncoders(_left);
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
		runForAllMotors(m -> {
			m.setSelectedSensorPosition(0);
		});
	};

	/**
	 * Average encoder values of a group
	 * @param motors Array of motors
	 * @return Average encoder value in MM
	 */
	@SuppressWarnings({"PMD.UseVarargs"})
	private double averageEncoders(WPI_TalonFX motors[]) {
		// Varargs not applicable here
		double sum = 0;
		for (WPI_TalonFX m : motors) {
			sum += m.getSelectedSensorPosition();
		}

		// Average & Convert to millimeters
		return (sum / motors.length) / UPMM;
	}

	/**
	 * Run function for all motors
	 * @param func Function to run
	 */
	private void runForAllMotors(Consumer<WPI_TalonFX> func) {
		for (WPI_TalonFX lm : _left) {
			func.accept(lm);
		}
		for (WPI_TalonFX lm : _right) {
			func.accept(lm);
		}
	}

}
