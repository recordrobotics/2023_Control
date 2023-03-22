// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger;

import java.util.ArrayList;
import java.util.List;

import org.recordrobotics.charger.commands.auto.AutoDrive;
import org.recordrobotics.charger.commands.auto.FullAutoTest;

import org.recordrobotics.charger.commands.auto.TrajectoryPresets;

import org.recordrobotics.charger.commands.auto.ParallelFullAuto;
import org.recordrobotics.charger.commands.auto.TrajectoryPresets;
import org.recordrobotics.charger.commands.manual.ManualClaw;
import org.recordrobotics.charger.commands.manual.ManualArm;
import org.recordrobotics.charger.commands.manual.ManualDrive;
import org.recordrobotics.charger.commands.dash.DashRunFunc;
import org.recordrobotics.charger.control.DoubleControl;
import org.recordrobotics.charger.control.IControlInput;
import org.recordrobotics.charger.control.SingleControl;
import org.recordrobotics.charger.subsystems.*;
import org.recordrobotics.charger.util.Pair;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
@SuppressWarnings({"PMD.SingularField","PMD.UnusedPrivateField"})
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private TrajectoryPresets _trajectoryPresets;
	private IControlInput _controlInput;
	//private Claw _claw;
	private Drive _drive;
	private DifferentialDrivePoseEstimator _estimator;
	private DifferentialDriveKinematics _kinematics;
	private ArrayList<Trajectory> _trajectories;
	private NavSensor _navSensor;
	private Vision _vision;
	//private Arm _arm;
	private PIDController _pid1;
	private PIDController _pid2;

	// Commands
	private List<Pair<Subsystem, Command>> _teleopPairs;
	private Command _autoCommand;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the button bindings
		_controlInput = new SingleControl((RobotMap.Control.SINGLE_GAMEPAD));
		_drive = new Drive();
		_navSensor = new NavSensor();
		//_claw = new Claw();
		//_arm = new Arm();
		_pid1 = new PIDController(0, 0, 0);
		_pid2 = new PIDController(0, 0, 0);

		_vision = new Vision();
		_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22));//This value should be confirmed when possible
		_estimator = new DifferentialDrivePoseEstimator(_kinematics, new Rotation2d(_navSensor.getYaw()), _drive.getLeftEncoder(), _drive.getRightEncoder(), new Pose2d(2.54, 4.65, new Rotation2d(0))); //The default standard deviations of the model states are 0.02 meters for x, 0.02 meters for y, and 0.01 radians for heading. The default standard deviations of the vision measurements are 0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
		//TODO: set an initial pose

		_trajectoryPresets = new TrajectoryPresets();
		_trajectories = _trajectoryPresets.SpinSpin9000();

		initTeleopCommands();
		initDashCommands();
	}

	private void initTeleopCommands() {
		_teleopPairs = new ArrayList<>();
		_teleopPairs.add(new Pair<Subsystem, Command>(_drive, new ManualDrive(_drive, _controlInput)));
		//_teleopPairs.add(new Pair<Subsystem, Command>(_claw, new ManualClaw(_claw, _controlInput)));
		//_teleopPairs.add(new Pair<Subsystem, Command>(_arm, new ManualArm(_arm, _controlInput, _pid1, _pid2)));
	}

	private void initDashCommands() {
		ShuffleboardTab tab = Shuffleboard.getTab(Constants.COMMANDS_TAB);
		tab.add("Single Control", new DashRunFunc(this::singleControl));
		tab.add("Double Control", new DashRunFunc(this::doubleControl));
	}

	/**
	 * Executes teleop commands
	 */	
	public void teleopInit() {
		for (Pair<Subsystem, Command> c : _teleopPairs) {
			c.getKey().setDefaultCommand(c.getValue());
		}
	}

	public Command getAutonomousCommand() {
		
		// Create a voltage constraint to ensure we don't accelerate too fast
		var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                Constants.DriveConstants.ksVolts,
                Constants.DriveConstants.kvVoltSecondsPerMeter,
                Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
				Constants.DriveConstants.kDriveKinematics,
            8);

		// Create config for trajectory
		TrajectoryConfig config =
			new TrajectoryConfig(
					Constants.AutoConstants.kMaxSpeedMetersPerSecond,
					Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(Constants.DriveConstants.kDriveKinematics)
				// Apply the voltage constraint
				.addConstraint(autoVoltageConstraint);

		// An example trajectory to follow.  All units in meters.
		Trajectory exampleTrajectory =
			TrajectoryGenerator.generateTrajectory(
				// Start at the origin facing the +X direction
				new Pose2d(0, 0, new Rotation2d(0)),
				// Pass through these two interior waypoints, making an 's' curve path
				List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
				// End 3 meters straight ahead of where we started, facing forward
				new Pose2d(3, 0, new Rotation2d(0)),
				// Pass config
				config);

		RamseteCommand ramseteCommand =
			new RamseteCommand(
				exampleTrajectory,
				_estimator.getEstimatedPosition(),
				new RamseteController(),
				new SimpleMotorFeedforward(
					Constants.DriveConstants.ksVolts,
					Constants.DriveConstants.kvVoltSecondsPerMeter,
					Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
				Constants.DriveConstants.kDriveKinematics,
				_drive::getWheelSpeeds,
				new PIDController(0, 0, 0),
				new PIDController(0, 0, 0),
				// RamseteCommand passes volts to the callback
			 	_drive::tankDriveVolts,
				_drive);

		// Reset odometry to the starting pose of the trajectory.
		_estimator.resetPosition(new Rotation2d(_navSensor.getYaw()), _drive.getLeftEncoder(), _drive.getRightEncoder(),  exampleTrajectory.getInitialPose());

		// Run path following command, then stop at the end.
		return ramseteCommand.andThen(() -> _drive.tankDriveVolts(0, 0));}
	/**
	 * Set control scheme to Single
	 */
	private void singleControl() {
		resetCommands();
		_controlInput = new SingleControl(RobotMap.Control.SINGLE_GAMEPAD);
		initTeleopCommands();
		teleopInit();
	}

	/**
	 * Set control scheme to Double
	 */
	private void doubleControl() {
		resetCommands();
		_controlInput = new DoubleControl(RobotMap.Control.DOUBLE_GAMEPAD_1,
			RobotMap.Control.DOUBLE_GAMEPAD_2);
		initTeleopCommands();
		teleopInit();
	}

	/**
	 * Clear commands
	 */
	public void resetCommands() {
		CommandScheduler.getInstance().cancelAll();
	}
}
