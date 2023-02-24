package org.recordrobotics.charger.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import org.recordrobotics.charger.RobotMap;

public class Gyroscope extends SubsystemBase {

	private ADXRS450_Gyro _gyro = new ADXRS450_Gyro(RobotMap.Gyro.GYRO_PORT);

	public double getDeg(){
		return _gyro.getAngle();
	}

	public double getRate(){
		return _gyro.getRate();
	}

	public void _gyroCalib() {
		_gyro.calibrate();
		_gyro.reset();
	}

	public void _gyroReset() {
		_gyro.reset();
	}
}
