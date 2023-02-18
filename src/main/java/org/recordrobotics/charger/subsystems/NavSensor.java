package org.recordrobotics.charger.subsystems;


import com.kauailabs.navx.frc.AHRS;

import org.recordrobotics.charger.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NavSensor extends SubsystemBase {
	public AHRS _nav = new AHRS(edu.wpi.first.wpilibj.I2C.Port.kOnboard);

	public NavSensor(){
		_nav.calibrate();
		ShuffleboardTab tab = Shuffleboard.getTab(Constants.DATA_TAB);
		tab.add("Pitch", _nav.getPitch()).getEntry();
		tab.add("Roll", _nav.getRoll()).getEntry();
		tab.add("Yaw", _nav.getYaw()).getEntry();
	}

	double getPitch() {
		return _nav.getPitch();
	}

	double getRoll() {
		return _nav.getRoll();
	}

	double getYaw() {
		return _nav.getYaw();
	}

	double getDisplacementX() {
		return _nav.getDisplacementX();
	}

	double getDisplacementY() {
		return _nav.getDisplacementY();
	}

	double getDisplacementZ() {
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
