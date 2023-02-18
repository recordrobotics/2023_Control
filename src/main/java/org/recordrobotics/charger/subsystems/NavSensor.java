package org.recordrobotics.charger.subsystems;


import com.kauailabs.navx.frc.AHRS;

//import org.recordrobotics.charger.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NavSensor extends SubsystemBase {
	public AHRS _nav = new AHRS(edu.wpi.first.wpilibj.I2C.Port.kOnboard);

	public NavSensor(){
		ShuffleboardTab tab = Shuffleboard.getTab(null/*Constants.DATA_TAB*/);
		tab.add("Pitch", _nav.getPitch());
	}

	public double getPitch() {
		return _nav.getPitch();
	}

	public double getRoll() {
		return _nav.getRoll();
	}

	public double getYaw() {
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
