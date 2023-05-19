package org.recordrobotics.Mitocondrion.commands.auto;

import org.recordrobotics.Mitocondrion.subsystems.Drive;
import org.recordrobotics.Mitocondrion.subsystems.NavSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ChargeStationBalance extends CommandBase {
	private PIDController _pid;
	private Drive _drive;
	private NavSensor _nav;
	//private SimpleMotorFeedforward _feedForward;
	private double encoderAtZero;
	
	private double _PIDTolerance = 0.005;
	private double _angleTolerance = 0.0524;


	/**
	 * sets up PID controller with p,i, and d values
	 * sets tolerance for the pid controller
	 * TODO: calibrate PID controller
	 * @param drive drivetrain object
	 * @param nav navigation sensor object, used here as a vertical gyro
	 */
	public ChargeStationBalance(Drive drive, NavSensor nav){
		_pid = new PIDController(11, 0, 0);
		//_feedForward = new SimpleMotorFeedforward(0, 0);
		_drive = drive;
		_nav = nav;
		_pid.setTolerance(_PIDTolerance);
		if (-1*_angleTolerance <= _nav.getPitch() && _nav.getPitch() <= _angleTolerance){
			encoderAtZero = (_drive.getLeftEncoder() + _drive.getRightEncoder())/2;
		}

	}

	/**
	 * runs the pid controller and then runs the drive motors accordingly
	 */
	@Override
	public void execute() {
		
		double _speed = _pid.calculate((_drive.getLeftEncoder() + _drive.getRightEncoder())/2, encoderAtZero);

		// Checks for max
		double SPEED_THRESHOLD = 0.45;
		if (_speed > SPEED_THRESHOLD) {
			System.out.println("MAX, was originally: " + _speed);
			_speed = SPEED_THRESHOLD;
		}
		// Prints
		System.out.println("Speed: " + _speed);
		SmartDashboard.putNumber("Speed", _speed);
		SmartDashboard.putNumber("Pitch", _nav.getPitch());
		SmartDashboard.putNumber("position", (_drive.getLeftEncoder() + _drive.getRightEncoder())/2);
		SmartDashboard.putNumber("0 position", encoderAtZero);
		//System.out.println("pitch " + _nav.getPitch());

		// Moves
		_drive.move(_speed, 0);//was negative
		}

	/**
	 * ends the command if the pid controller is at the intended point within the correct tolerance
	 */
	@Override
	public boolean isFinished() {
		//Ends the command when the robot is at it's intended position
		return false;
	}

	@Override
	public void end(boolean interrupted) {
		//Shuts off the drive motors when the command ends
		_drive.move(0, 0);
	}
}