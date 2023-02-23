// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger;

import java.util.ArrayList;
import java.util.List;

import org.recordrobotics.charger.commands.manual.ManualClaw;
//import org.recordrobotics.charger.commands.manual.ManualDrive;
//import org.recordrobotics.charger.control.DoubleControl;
import org.recordrobotics.charger.control.IControlInput;
import org.recordrobotics.charger.control.SingleControl;
import org.recordrobotics.charger.subsystems.*;
import org.recordrobotics.charger.util.Pair;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private IControlInput _controlInput;
	//private Drive _drive;
	private Claw _claw;

	// Commands
	@SuppressWarnings({"PMD.SingularField"})
	private List<Pair<Subsystem, Command>> _teleopPairs;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// Configure the button bindings
		// _controlInput = new DoubleControl(Constants.Control.DOUBLE_GAMEPAD_1, Constants.Control.DOUBLE_GAMEPAD_2);
		_controlInput = new SingleControl(Constants.Control.LEGACY_GAMEPAD);
		//_drive = new Drive();
		_claw = new Claw();

		initTeleopCommands();
	}

	private void initTeleopCommands() {
		_teleopPairs = new ArrayList<>();
		//_teleopPairs.add(new Pair<Subsystem, Command>(_drive, new ManualDrive(_drive, _controlInput)));
		_teleopPairs.add(new Pair<Subsystem, Command>(_claw, new ManualClaw(_claw, _controlInput)));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		// An ExampleCommand will run in autonomous
		return null;
	}

	/**
	 * Executes teleop commands
	 */
	public void teleopInit() {
		for (Pair<Subsystem, Command> c : _teleopPairs) {
			c.getKey().setDefaultCommand(c.getValue());
		}
	}
}
