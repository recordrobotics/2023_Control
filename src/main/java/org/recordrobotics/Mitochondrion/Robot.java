// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.Mitochondrion;
import org.recordrobotics.Mitochondrion.subsystems.Drive;
import org.recordrobotics.Mitochondrion.subsystems.Vision;
import org.recordrobotics.Mitochondrion.subsystems.NavSensor;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@SuppressWarnings({"PMD.SystemPrintln", "PMD.FieldNamingConventions"})
public class Robot extends TimedRobot {
	private RobotContainer _robotContainer;
	private Command _autonomousCommand;
	//private Vision _vision;
	private Timer _timer;
	private DifferentialDrivePoseEstimator _estimator;
	private Drive _drive;
	private DifferentialDriveKinematics _kinematics = new DifferentialDriveKinematics(22);
	private NavSensor _navSensor;

	@SuppressWarnings("PMD.SingularField")
	private Field2d field;

	/**
	 * Robot initialization
	 */

	@Override
	public void robotInit() {
		//System.out.println("Robotinit");
		// Create container
		_robotContainer = new RobotContainer();
		field = new Field2d();
		//_vision = new Vision();
		//_timer = new Timer();
		//_navSensor = new NavSensor();
		//_drive = new Drive();
		//_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22));//This value should be confirmed when possible
		//_estimator = new DifferentialDrivePoseEstimator(_kinematics, new Rotation2d(_navSensor.getYaw()), _drive.getLeftEncoder(), _drive.getRightEncoder(), new Pose2d(2.54, 4.65, new Rotation2d(0))); //The default standard deviations of the model states are 0.02 meters for x, 0.02 meters for y, and 0.01 radians for heading. The default standard deviations of the vision measurements are 0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
		//SmartDashboard.putData(field);
	}


	/**
	* Runs every robot tick
	*/
	@Override
	public void robotPeriodic() {
		//System.out.println("Robot periodic");
		// Run command scheduler
		CommandScheduler.getInstance().run();
	}

	/**
	 * Runs when robot enters disabled mode
	 */
	@Override
	public void disabledInit() {
		_robotContainer.resetCommands();
	}

	/**
	 * Runs every tick during disabled mode
	 */
	@Override
	public void disabledPeriodic() {
		//System.out.println("Disabled periodic");
		// TODO
	}

	@Override
	public void disabledExit() {
		_robotContainer.disabledExit();
	}

	/**
	 * Runs when robot enters auto mode
	 */
	@Override
	public void autonomousInit() {
		System.out.println("Autonomous Init");
		_autonomousCommand = _robotContainer.getAutonomousCommand();


		// schedule the autonomous command (example)
		if (_autonomousCommand != null) {
			_autonomousCommand.schedule();
		}
	}

	/**
	 * Runs every tick during auto mode
	 */
	@Override
	public void autonomousPeriodic() {
		//placeholder
	}

	/**
	 * Runs when robot enters teleop mode
	 */
	@Override
	public void teleopInit() {
		System.out.println("Teleop init");
		// TODO
		if (_autonomousCommand != null) {
			_autonomousCommand.cancel();
		}
		_robotContainer.teleopInit();
	}

	/**
	* Runs every tick in teleop mode
	*/
	@Override
	public void teleopPeriodic() {
		//placholder
		/*if (Vision.checkForTarget(_vision.camera)){
			double[] globalPose = Vision.estimateGlobalPose(_vision.camera);
			Pose2d visPose = new Pose2d(globalPose[0], globalPose[1], new Rotation2d(globalPose[2]));
			_estimator.addVisionMeasurement(visPose, _timer.get());
		}*/
		
		// Calculates angle measurements given encoder values
		/*Rotation2d nav_sensor_spoof = new Rotation2d(
			((-1*_drive.getRightEncoder()/1000)-(-1*_drive.getLeftEncoder()/1000))/(2*Units.inchesToMeters(11))
		);

		_estimator.update(nav_sensor_spoof, -1*_drive.getLeftEncoder()/1000, -1*_drive.getRightEncoder()/1000);
		
		Pose2d pose = _estimator.getEstimatedPosition();
		*/
		//System.out.print("yaw " + _navSensor.getYaw());
		//System.out.println("y " + _navSensor.getDisplacementY());
		//System.out.println("x " + _navSensor.getDisplacementX());
		//System.out.println("encoders " + _drive.getLeftEncoder() + ", " + _drive.getRightEncoder());

		//System.out.println(pose.getX() + ", " + pose.getY() + ", " + pose.getRotation().getRadians());
		}

	@Override
	public void testInit() {
		System.out.println("Test init");
		// TODO
		if (_autonomousCommand != null) {
			_autonomousCommand.cancel();
		}
		_robotContainer.testInit();		
	}

	@Override
	public void testPeriodic() {
		_robotContainer.testPeriodic();
	}

	}
