package org.recordrobotics.charger.commands.auto;

import org.recordrobotics.charger.subsystems.Drive;
import org.recordrobotics.charger.subsystems.NavSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ChargeStationBalance extends CommandBase {
	private PIDController _pid;
	private Drive _drive;
	private NavSensor _nav;

	private double bkp = 0;
	private double bki = 0;
	private double bkd = 0;

	private double _tolerance = 3;

	/**
	 * sets up PID controller with p,i, and d values
	 * sets tolerance for the pid controller
	 * TODO: calibrate PID controller
	 * @param drive drivetrain object
	 * @param nav navigation sensor object, used here as a vertical gyro
	 */
	public ChargeStationBalance(Drive drive, NavSensor nav){
		_pid = new PIDController(bkp, bki, bkd);
		_drive = drive;
		_nav = nav;
		_pid.setTolerance(_tolerance);

	}

	/**
	 * runs the pid controller and then runs the drive motors accordingly
	 */
	@Override
	public void execute() {
		double _speed = _pid.calculate(_nav.getPitch(), 0);
		_drive.move(_speed, 0);
	}

	/**
	 * ends the command if the pid controller is at the intended point within the correct tolerance
	 */
	@Override
	public boolean isFinished() {
		//Ends the command when the robot is at it's intended position
		return _pid.atSetpoint();
	}

	@Override
	public void end(boolean interrupted) {
		//Shuts off the drive motors when the command ends
		_drive.move(0, 0);
	}
}