package org.recordrobotics.charger.subsystems;


import org.recordrobotics.charger.Constants;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NavSensor extends SubsystemBase {

	private GenericEntry _pitchEntry;

	public AHRS _nav = new AHRS(edu.wpi.first.wpilibj.I2C.Port.kMXP);
	
	public NavSensor(){
		ShuffleboardTab tab = Shuffleboard.getTab(Constants.DATA_TAB);
		_pitchEntry = tab.add("Pitch", _nav.getPitch()).getEntry();
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

	@Override
	public void periodic() {
		_pitchEntry.setDouble(getPitch());
	}
}
