// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger;

import java.util.ArrayList;
import org.recordrobotics.charger.subsystems.Drive;
import org.recordrobotics.charger.subsystems.NavSensor;
import org.recordrobotics.charger.subsystems.Vision;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
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
	private Trajectory trajectory;
	Timer timer = new Timer();

	private final RamseteController ramseteController = new RamseteController();

	@SuppressWarnings("PMD.SingularField")
	private Field2d field;

	DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(20.75));
	DifferentialDrivePoseEstimator estimator = new DifferentialDrivePoseEstimator(kinematics, new Rotation2d(gyro.getYaw()), drive.getLeftEncoder(), drive.getRightEncoder(), new Pose2d(1.5, 3.0, new Rotation2d(3.0))); //The default standard deviations of the model states are 0.02 meters for x, 0.02 meters for y, and 0.01 radians for heading. The default standard deviations of the vision measurements are 0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
	//TODO: figure out initial pose strategy above

	TrajectoryConfig config = new TrajectoryConfig(2, 1); //DEFINE MAX VELOCITY AND ACCELERATION HERE

	public Trajectory getTrajectory(Pose2d start, TrajectoryConfig config) {
		Rotation2d angle = new Rotation2d(Math.PI/4);
		Pose2d ball = new Pose2d(Units.inchesToMeters(277.949), Units.inchesToMeters(36.000), angle);
		Pose2d score = new Pose2d(Units.inchesToMeters(69.438), Units.inchesToMeters(42.000), angle);

		ArrayList<Pose2d> waypoints = new ArrayList<>();
		waypoints.add(start);
		waypoints.add(ball);
		waypoints.add(score);

		return TrajectoryGenerator.generateTrajectory(waypoints, config);
		}

	/**
	 * Robot initialization
	 */

	@Override
	public void robotInit() {
		//System.out.println("Robotinit");
		// Create container
		_robotContainer = new RobotContainer();
		var trajectory = getTrajectory(new Pose2d(1.5, 3.0, new Rotation2d(3.0)), config);//TODO: starting pose
		field = new Field2d();
		SmartDashboard.putData(field);
		field.getObject("traj").setTrajectory(trajectory);


		System.out.println("LMAOOO (end of robot init)");
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
		//TODO: set an initial value for the estimator here somehow?
	}

	/**
	 * Runs every tick during auto mode
	 */
	@Override
	public void autonomousPeriodic() {
		System.out.println("Autonomous periodic");
		double[] globalPose = Vision.getVisionPoseEstimate(vision.camera, vision.robotToCam);	
		
		
		System.out.println(globalPose[0] + "\n" + globalPose[1] + "\n" + globalPose[2]);


		Pose2d visPose = new Pose2d(globalPose[0], globalPose[1], new Rotation2d(globalPose[2]));

		System.out.println("End of creation of visPose object");
		System.out.println("Timer: " + Timer.getFPGATimestamp());

		estimator.addVisionMeasurement(visPose, Timer.getFPGATimestamp());
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

//}
