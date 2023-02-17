package org.recordrobotics.charger.subsystems;


import com.kauailabs.navx.frc.AHRS;

import org.recordrobotics.charger.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NavSensor extends SubsystemBase {
	public AHRS _nav = new AHRS();

	public NavSensor(){
		ShuffleboardTab tab = Shuffleboard.getTab(Constants.DATA_TAB);
		tab.add("Pitch", _nav.getPitch());
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
