// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger;

import java.util.ArrayList;
import java.util.List;

import org.recordrobotics.charger.commands.auto.FullAutoSequence;
import org.recordrobotics.charger.commands.auto.AutoDrive;
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
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	@SuppressWarnings({"PMD.SingularField","AvoidDuplicateLiterals"})
	private IControlInput _controlInput;
	private Drive _drive;
	@SuppressWarnings({"PMD.SingularField","PMD.UnusedPrivateField"})
	private DifferentialDrivePoseEstimator _estimator;
	@SuppressWarnings({"PMD.SingularField"})
	private DifferentialDriveKinematics _kinematics;
	@SuppressWarnings({"PMD.SingularField","PMD.UnusedLocalVariable"})
	private Trajectory _trajcetory;
	@SuppressWarnings({"PMD.SingularField"})
	private NavSensor _navSensor;
	// Commands
	@SuppressWarnings({"PMD.SingularField"})
	private List<Pair<Subsystem, Command>> _teleopPairs;
	private Command _autoCommand;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the button bindings
		_controlInput = new DoubleControl(RobotMap.Control.DOUBLE_GAMEPAD_1, RobotMap.Control.DOUBLE_GAMEPAD_2);
		_drive = new Drive();
		_navSensor = new NavSensor();

		_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(20.75));
		_estimator = new DifferentialDrivePoseEstimator(_kinematics, new Rotation2d(_navSensor.getYaw()), _drive.getLeftEncoder(), _drive.getRightEncoder(), null); //The default standard deviations of the model states are 0.02 meters for x, 0.02 meters for y, and 0.01 radians for heading. The default standard deviations of the vision measurements are 0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.

		//var trajectory = Trajectories.getTrajectory(null, Trajectories.config);//TODO: starting pose
		@SuppressWarnings({"PMD.UnusuedPrivateField"})
		Trajectory _trajectory = Trajectories.testTrajectory(new Pose2d(1.22743, 2.748026, new Rotation2d(0)), Trajectories.config);
		//var trajectory = Trajectories.visTestTrajectory(new Pose2d(1.62743, 2.748026, new Rotation2d(Math.PI)), Trajectories.config);

		//TODO: figure out initial pose strategy above

		initTeleopCommands();
		initDashCommands();
		initAutoCommand();
	}

	private void initTeleopCommands() {
		_teleopPairs = new ArrayList<>();
		_teleopPairs.add(new Pair<Subsystem, Command>(_drive, new ManualDrive(_drive, _controlInput)));
	}

	private void initAutoCommand() {
		_autoCommand = new AutoDrive(_drive, 0.45, 1000);
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
		return _autoCommand;
	}

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
