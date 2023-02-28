// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger;

import org.recordrobotics.charger.subsystems.Drive;
import org.recordrobotics.charger.subsystems.NavSensor;
import org.recordrobotics.charger.subsystems.Trajectories;
import org.recordrobotics.charger.subsystems.Vision;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
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


	/*placeholder description*/
	private final Drive drive = new Drive();
	NavSensor gyro = new NavSensor();
	Vision vision = new Vision();
	public Trajectory trajectory;//This was private in the example code, I changed it as it may be part of an issue
	Timer timer = new Timer();

	int autoPhase = 0;

	private final RamseteController ramseteController = new RamseteController();

	@SuppressWarnings("PMD.SingularField")
	private Field2d field;

	DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22));
	DifferentialDrivePoseEstimator estimator = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(gyro.getYaw()), drive.getLeftEncoder(), drive.getRightEncoder(), new Pose2d(1.22743, 2.748026, new Rotation2d(0))); //The default standard deviations of the model states are 0.02 meters for x, 0.02 meters for y, and 0.01 radians for heading. The default standard deviations of the vision measurements are 0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
	//TODO: figure out initial pose strategy above

	/**
	 * Robot initialization
	 */

	@Override
	public void robotInit() {
		//System.out.println("Robotinit");
		// Create container
		_robotContainer = new RobotContainer();
		//var trajectory = Trajectories.getTrajectory(null, Trajectories.config);//TODO: starting pose
		trajectory = Trajectories.testTrajectory(new Pose2d(1.22743, 2.748026, new Rotation2d(0)), Trajectories.config);
		//var trajectory = Trajectories.visTestTrajectory(new Pose2d(1.62743, 2.748026, new Rotation2d(Math.PI)), Trajectories.config);
		field = new Field2d();
		SmartDashboard.putData(field);
		field.getObject("traj").setTrajectory(trajectory);
		vision.camera.setPipelineIndex(2);
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
		System.out.println("Disabled init");
		//_robotContainer.resetCommands();
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
		timer.start();
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
		System.out.println("Autonomous periodic");

		if (Vision.checkForTarget(vision.camera, vision.robotToCam)){
			double[] globalPose = Vision.getVisionPoseEstimate(vision.camera, vision.robotToCam);
			Pose2d visPose = new Pose2d(globalPose[0], globalPose[1], new Rotation2d(globalPose[2]));
			estimator.addVisionMeasurement(visPose, timer.get());
		}
		estimator.update(new Rotation2d(gyro.getYaw()), drive.getLeftEncoder(), drive.getRightEncoder());
		if (timer.get() < trajectory.getTotalTimeSeconds()) {
			// Get the desired pose from the trajectory.
			var desiredPose = trajectory.sample(timer.get());

			// Get the reference chassis speeds from the Ramsete controller.
			var refChassisSpeeds = ramseteController.calculate(estimator.getEstimatedPosition(), desiredPose);

			// Set the linear and angular speeds.
			drive.move(refChassisSpeeds.vxMetersPerSecond, refChassisSpeeds.omegaRadiansPerSecond);
		} else {
			drive.move(0, 0);
		}
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
