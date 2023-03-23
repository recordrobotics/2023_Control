package org.recordrobotics.charger.subsystems;

import org.recordrobotics.charger.Constants;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;

import edu.wpi.first.math.util.Units;

import java.lang.*;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavSensor extends SubsystemBase {
	AHRS _nav;

	
	//public AHRS _nav = new AHRS(SerialPort.Port.kUSB1);

	//public NavSensor(){
	//	ShuffleboardTab tab = Shuffleboard.getTab(Constants.DATA_TAB);
	//	tab.add("Pitch", _nav.getPitch());

	//

	//System.out.println(_nav.getPitch(), _nav.getRoll(), _nav.getYaw());

	public NavSensor() {

		_nav = new AHRS(SerialPort.Port.kUSB1);

		_nav.reset();
		_nav.resetDisplacement();

		//_nav.enableBoardlevelYawReset​(true);
	}

	public double getPitch() {
		double pitch = _nav.getRoll();
		return Units.degreesToRadians(pitch);
	}

	public double getRoll() {
		double roll = _nav.getPitch();
		return Units.degreesToRadians(-1*roll);
	}

	public double getYaw() {
		double yaw = _nav.getYaw();
		return Units.degreesToRadians(-1*yaw);
	}

	//None of the below are guarenteed to work (weird axis changes)

	public double getDisplacementX() {
		return _nav.getDisplacementX();
	}

	public double getDisplacementY() {
		return _nav.getDisplacementY();
	}

	public double getDisplacementZ() {
		return _nav.getDisplacementZ();
	}

	void resetAngle() {
		_nav.reset();
	}

	void resetDisplacement() {
		_nav.resetDisplacement();
	}

	void resetAll(){
		resetAngle();
		resetDisplacement();
	}
}
