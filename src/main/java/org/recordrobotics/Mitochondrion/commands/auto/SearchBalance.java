package org.recordrobotics.Mitochondrion.commands.auto;

import org.recordrobotics.Mitochondrion.subsystems.Drive;
import org.recordrobotics.Mitochondrion.subsystems.NavSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SearchBalance extends CommandBase {
    
    private PIDController _pid;
    private Drive _drive;
    private NavSensor _nav;
    private double _farBound;
    private double _nearBound;
    private double _tolerance;
    private double _speed;
    private double _rampIncline;
    private int _step;
    private double _center;
    private double _PIDTolerance = 0.005;

    public SearchBalance(Drive drive, NavSensor nav){

        _pid = new PIDController(11, 0, 0);
        _drive = drive;
        _nav = nav;
        _speed = 0.5; //This is NOT in m/s, this is the motor input, 0 < v < 1
        _tolerance = 0.0524; // radians
        _rampIncline = 0;// TODO: find this
        _step = 0;
        _center = 0;
        _pid.setTolerance(_PIDTolerance);
    
    }

    @Override
    public void execute(){
        if (_step == 0){
        while (_nav.getPitch() + _tolerance < _rampIncline && _nav.getPitch() - _tolerance > _rampIncline){//pitching forward as positive
            _drive.move(-1*_speed, 0);
        }

        _farBound = (_drive.getLeftEncoder() + _drive.getRightEncoder())/2;

        while (-1*_nav.getPitch() + _tolerance < _rampIncline && -1*_nav.getPitch() - _tolerance > _rampIncline){
            _drive.move(_speed, 0);
        }

        _nearBound = (_drive.getLeftEncoder() + _drive.getRightEncoder())/2;

        _step += 1;
        }

        if (_step == 1){
            _center = (_farBound + _nearBound)/2;

            double _speed = _pid.calculate((_drive.getLeftEncoder() + _drive.getRightEncoder())/2, _center);

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
		SmartDashboard.putNumber("0 position", _center);
		//System.out.println("pitch " + _nav.getPitch());

		// Moves
		_drive.move(_speed, 0);//was negative

        }
    }

    @Override
    public boolean isFinished(){
        return false; // TODO:

    }

}
