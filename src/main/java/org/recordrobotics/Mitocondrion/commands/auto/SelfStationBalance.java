package org.recordrobotics.Mitocondrion.commands.auto;

import java.lang.reflect.Array;

import javax.lang.model.element.ModuleElement.Directive;

import org.recordrobotics.Mitocondrion.subsystems.Drive;
import org.recordrobotics.Mitocondrion.subsystems.NavSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SelfStationBalance extends CommandBase {
	private PIDController _pid;
	private Drive _drive;
	private NavSensor _nav;

    public boolean _are_we_done_yet;
	public double _measurement_list[];
	public double _derivative_list[];
	public int _moving_average_counter;
	public double _nav_offset;



	/**
	 * sets up PID controller with p,i, and d values
	 * sets tolerance for the pid controller
	 * TODO: calibrate PID controller
	 * @param drive drivetrain object
	 * @param nav navigation sensor object, used here as a vertical gyro
	 */
	public SelfStationBalance(Drive drive, NavSensor nav, double nav_offset){
		_drive = drive;
		_nav = nav;

		_are_we_done_yet = false;
		_measurement_list = new double [10];
		_derivative_list = new double [9];
		_nav_offset = nav_offset;

		// sets all values to 0
		for (int i=0;i<_measurement_list.length;i++) {
			_measurement_list[i] = 0;
		}

	}

    

	/**
	 * runs the pid controller and then runs the drive motors accordingly
	 */
	@Override
	public void execute() {

		//Updates rolling measurement array
		for (int i=1;i<_measurement_list.length;i++) {
			_measurement_list[i-1] = _measurement_list[i];
		}
		_measurement_list[_measurement_list.length-1] = -1*(_nav.getPitch() - _nav_offset); //-1 because facing backwards

		// Takes average
		int sum = 0;
		for (double value : _measurement_list) {
			sum += value;}
		double average = sum/_measurement_list.length;
		//

		// Fills in derivative list
		for (int i=0;i<_derivative_list.length;i++) {
			_derivative_list[i] = (_measurement_list[i+1] - _measurement_list[i])/0.02;}
		//
	
		// Finds average dPitch/dTime
		int d_sum = 0;
		for (double d_value : _derivative_list) {
			d_sum += d_value;}
		double d_average = d_sum/_derivative_list.length;
		//

		// marks done if the moving average of pitch is less than 
		if (average < 0.04 /*5 degrees = 0.0873*/) {
			System.out.println("degree");
			_are_we_done_yet = true;}
		// marks done if the moving average of the derivative is less than 0
		if (d_average < 0 /*5 degrees*/) {
			System.out.println("deriv");
			_are_we_done_yet = true;}
		//

		// Sets speed
		_drive.move(-0.2, 0);
		//
	}

	/**
	 * ends the command if the pid controller is at the intended point within the correct tolerance
	 */
	@Override
	public boolean isFinished() {
		//Ends the command when the robot is at it's intended position
		return _are_we_done_yet;
	}

	@Override
	public void end(boolean interrupted) {
		//Shuts off the drive motors when the command ends
		_drive.move(0, 0);
	}
}
