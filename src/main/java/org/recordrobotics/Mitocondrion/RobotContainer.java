// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.Mitocondrion;

import java.util.ArrayList;
import java.util.List;

import org.recordrobotics.Mitocondrion.commands.auto.AutoDrive;
import org.recordrobotics.Mitocondrion.commands.auto.ChargeStationBalance;
import org.recordrobotics.Mitocondrion.commands.auto.FullAuto;
import org.recordrobotics.Mitocondrion.commands.auto.MoveToChargeStation;
import org.recordrobotics.Mitocondrion.commands.auto.SelfStationBalance;
import org.recordrobotics.Mitocondrion.commands.auto.SimpleScoreAndTaxi;
import org.recordrobotics.Mitocondrion.commands.auto.VisionBalance;
/*import org.recordrobotics.Mitocondrion.commands.auto.SimpleScoreTaxiDock;
import org.recordrobotics.Mitocondrion.commands.auto.TrajectoryPresets;

import org.recordrobotics.Mitocondrion.commands.auto.TestPreset;
*/
import org.recordrobotics.Mitocondrion.commands.manual.ManualClaw;
//import org.recordrobotics.Mitocondrion.commands.manual.ArmPosition;
import org.recordrobotics.Mitocondrion.commands.manual.ManualArm;
import org.recordrobotics.Mitocondrion.commands.manual.CompManualArm;
import org.recordrobotics.Mitocondrion.commands.manual.ManualDrive;
import org.recordrobotics.Mitocondrion.commands.dash.DashRunFunc;
import org.recordrobotics.Mitocondrion.control.DoubleControl;
import org.recordrobotics.Mitocondrion.control.IControlInput;
import org.recordrobotics.Mitocondrion.control.SingleControl;
import org.recordrobotics.Mitocondrion.subsystems.*;
import org.recordrobotics.Mitocondrion.util.Pair;
import org.recordrobotics.Mitocondrion.util.GetStartTime;


import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
	//private TrajectoryPresets _trajectoryPresets;
	private IControlInput _controlInput;
	private Claw _claw;
	private Drive _drive;
	private DifferentialDrivePoseEstimator _estimator;
	private DifferentialDriveKinematics _kinematics;
	private ArrayList<Trajectory> _trajectories;
	private NavSensor _navSensor;
	private Vision _vision;
	//private CompArm _compArm;
	private Arm _arm;
	private PIDController _pid1;
	private PIDController _pid2;
	public GetStartTime _GetStartTime;
	private RamseteController _ramsete;

	// Commands
	private List<Pair<Subsystem, Command>> _teleopPairs;
	//private Command _autoCommand;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		resetCommands();

		// Configure the button bindings
		_controlInput = new DoubleControl(RobotMap.Control.DOUBLE_GAMEPAD_1, RobotMap.Control.DOUBLE_GAMEPAD_2);
		_drive = new Drive();
		_navSensor = new NavSensor();
		//_claw = new Claw();
		//_compArm = new CompArm();
		//_arm = new Arm();
		_pid1 = new PIDController(0, 0, 0);
		_pid2 = new PIDController(0, 0, 0);

		_vision = new Vision();
		_kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(22));//This value should be confirmed when possible
		_estimator = new DifferentialDrivePoseEstimator(_kinematics, new Rotation2d(_navSensor.getYaw()), _drive.getLeftEncoder(), _drive.getRightEncoder(), new Pose2d(2.54, 4.65, new Rotation2d(0))); //The default standard deviations of the model states are 0.02 meters for x, 0.02 meters for y, and 0.01 radians for heading. The default standard deviations of the vision measurements are 0.1 meters for x, 0.1 meters for y, and 0.1 radians for heading.
		_ramsete = new RamseteController();
		//TODO: set an initial pose

		/*_trajectoryPresets = new TrajectoryPresets();
		_trajectories = _trajectoryPresets.testTraj2();
*/
		initTeleopCommands();
		initDashCommands();
	}

	private void initTeleopCommands() {
		_teleopPairs = new ArrayList<>();
		_teleopPairs.add(new Pair<Subsystem, Command>(_drive, new ManualDrive(_drive, _controlInput)));
		//_teleopPairs.add(new Pair<Subsystem, Command>(_compArm, new CompManualArm(_compArm, _controlInput)));
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
		//_arm.resetPID();
		for (Pair<Subsystem, Command> c : _teleopPairs) {
			c.getKey().setDefaultCommand(c.getValue());
		}
	}

	public Command getAutonomousCommand() {

		// Gets nav sensors offset
		double nav_offset = _navSensor.getPitch();

		_drive.resetEncoders(); // resets encoders
		//return new TrajectoryPresets(_vision, _drive, _pid2, _pid1, _trajectories, _estimator, _navSensor);//new ParallelFullAuto(_vision, _drive, _arm, _claw, _pid1, _pid2, _trajectories, _estimator, _navSensor)//


		//double auto_start_time = Timer.getFPGATimestamp();
		//return new SimpleScoreAndTaxi(_drive, _arm, _claw,  ArmPosition.SECOND);


		
		//return new FullAuto(_vision, _drive, _trajectories, _ramsete, _kinematics, _estimator, _navSensor, null, _claw);
		return new VisionBalance(_drive, _navSensor, _vision, new Pose2d(), _estimator);
		//return new SelfStationBalance(_drive, _navSensor, nav_offset);
		//return new FullAutoTest(_vision, _drive, _pid2, _pid1, _trajectories, _estimator, _navSensor, auto_start_time);//new ParallelFullAuto(_vision, _drive, _arm, _claw, _pid1, _pid2, _trajectories, _estimator, _navSensor)
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

	public void testInit() {}

	public void testPeriodic() {}

	/**
	 * Clear commands
	 */
	public void resetCommands() {
		CommandScheduler.getInstance().cancelAll();
	}
}
