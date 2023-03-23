package org.recordrobotics.charger.subsystems;

import org.recordrobotics.charger.Constants;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;


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

		//_nav.calibrate();

		//_nav.enableBoardlevelYawReset​(true);
	}

	public float getPitch() {
		return _nav.getPitch();
	}

	public float getRoll() {
		return _nav.getRoll();
	}

	public float getYaw() {
		return _nav.getYaw();
	}

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
