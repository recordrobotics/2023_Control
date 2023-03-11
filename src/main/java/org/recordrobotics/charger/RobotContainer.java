// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger;

import java.util.ArrayList;
import java.util.List;

import org.recordrobotics.charger.commands.auto.AutoDrive;
import org.recordrobotics.charger.commands.manual.ManualClaw;
import org.recordrobotics.charger.commands.manual.ManualArm;
import org.recordrobotics.charger.commands.manual.ManualDrive;
import org.recordrobotics.charger.commands.dash.DashRunFunc;
import org.recordrobotics.charger.control.DoubleControl;
import org.recordrobotics.charger.control.IControlInput;
import org.recordrobotics.charger.control.SingleControl;
import org.recordrobotics.charger.subsystems.*;
import org.recordrobotics.charger.util.Pair;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class RobotContainer {

	// Subsystems
	private IControlInput _controlInput;
	private Drive _drive;
	private Arm _arm;
	private Claw _claw;

	// TODO: move to drive
	// private DifferentialDrivePoseEstimator _estimator;
	// private DifferentialDriveKinematics _kinematics;

	// Auto/Vision
	// private List<Trajectory> _trajectories;
	// private TrajectoryPresets _trajectoryPresets;
	// private Vision _vision;
	// private NavSensor _navSensor;

	// PID
	private PIDController _pid1;
	private PIDController _pid2;

	// Commands
	private List<Pair<Subsystem, Command>> _teleopPairs;
	private Command _autoCommand;

	public RobotContainer() {
		// Create subsystems
		_controlInput = new DoubleControl(RobotMap.Control.DOUBLE_GAMEPAD_1, RobotMap.Control.DOUBLE_GAMEPAD_2);
		_drive = new Drive();
		_claw = new Claw();
		_arm = new Arm();

		// Auto/Vision
		// _navSensor = new NavSensor();
		// _vision = new Vision();
		//_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22));//This value should be confirmed when possible
		//_estimator = new DifferentialDrivePoseEstimator(_kinematics, new Rotation2d(_navSensor.getYaw()), _drive.getLeftEncoder(), _drive.getRightEncoder(), new Pose2d(2.54, 4.65, new Rotation2d(0))); //The default standard deviations of the model states are 0.02 meters for x, 0.02 meters for y, and 0.01 radians for heading. The default standard deviations of the vision measurements are 0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
		//TODO: set an initial pose
		// _trajectoryPresets = new TrajectoryPresets();
		// _trajectories = new ArrayList<Trajectory>();//TODO: replace this with whatever trajectory preset is relevant
		// _trajectories.add(_trajectoryPresets.testTraj());

		// PID
		_pid1 = new PIDController(0, 0, 0);
		_pid2 = new PIDController(0, 0, 0);

		// Setup commands
		initTeleopCommands();
		initDashCommands();
		initAutoCommand();
	}

	private void initTeleopCommands() {
		_teleopPairs = new ArrayList<>();
		_teleopPairs.add(new Pair<Subsystem, Command>(_drive, new ManualDrive(_drive, _controlInput)));
		_teleopPairs.add(new Pair<Subsystem, Command>(_claw, new ManualClaw(_claw, _controlInput)));
		_teleopPairs.add(new Pair<Subsystem, Command>(_arm, new ManualArm(_arm, _controlInput, _pid1, _pid2)));
	}

	private void initDashCommands() {
		ShuffleboardTab tab = Shuffleboard.getTab(Constants.COMMANDS_TAB);
		tab.add("Single Control", new DashRunFunc(this::singleControl));
		tab.add("Double Control", new DashRunFunc(this::doubleControl));
	}

	private void initAutoCommand() {
		_autoCommand = new AutoDrive(_drive, 0.4, 1750);
	}

	/**
	 * Executes teleop commands
	 */
	public void teleopInit() {
		for (Pair<Subsystem, Command> c : _teleopPairs) {
			c.getKey().setDefaultCommand(c.getValue());
		}
	}

	public Command getAutoCommand() {
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
