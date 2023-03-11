// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.recordrobotics.charger.subsystems.Drive;
import org.recordrobotics.charger.subsystems.NavSensor;
import org.recordrobotics.charger.subsystems.Vision;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;


@SuppressWarnings({"PMD.SystemPrintln", "PMD.FieldNamingConventions"})
public class Robot extends TimedRobot {
	private RobotContainer _robotContainer;
	private Command _autonomousCommand;

	// new imports
	private Vision _vision;
	private Drive _drive;
	private DifferentialDrivePoseEstimator _estimator;
	private NavSensor _nav;
	private Timer _timer;
	private DifferentialDriveKinematics _kinematics;

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
		SmartDashboard.putData(field);
		_vision = new Vision();
		_timer = new Timer();
		_drive = new Drive();
		_nav = new NavSensor();
		_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22));
		_estimator = new DifferentialDrivePoseEstimator(_kinematics, new Rotation2d(_nav.getYaw()), _drive.getLeftEncoder(), _drive.getRightEncoder(), new Pose2d(2.54, 4.65, new Rotation2d(0)));
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

	/**
	 * Runs when robot enters auto mode
	 */
	@Override
	public void autonomousInit() {
		System.out.println("Autonomous Init");
		//_autonomousCommand = _robotContainer.getAutonomousCommand();


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
		
		//System.out.println("WE GOT HERE");
		if (Vision.checkForTarget(_vision.camera)){
			double[] globalPose = Vision.estimateGlobalPose(_vision.camera);
			double timer_thing = _timer.getFPGATimestamp();

			System.out.println("VISION POSE: " + globalPose[0] + " " + globalPose[1] + " " + globalPose[2]);			
			System.out.println("TIMER: " + timer_thing);

			Pose2d visPose = new Pose2d(globalPose[0], globalPose[1], new Rotation2d(globalPose[2]));
			_estimator.addVisionMeasurement(visPose, timer_thing);

			//System.out.println("vision detected");
		}
		_estimator.update(new Rotation2d(_nav.getYaw()), _drive.getLeftEncoder(), _drive.getRightEncoder());

		Pose2d global_pose = _estimator.getEstimatedPosition();
		System.out.println("KALMAN POSE: " + global_pose.getX() + " " + global_pose.getY() + " " + global_pose.getRotation().getRadians());

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
		}


}