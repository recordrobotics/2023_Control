package org.recordrobotics.charger.subsystems;


import com.kauailabs.navx.frc.AHRS;

import org.recordrobotics.charger.Constants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class NavSensor extends SubsystemBase {
	public AHRS _nav = new AHRS(edu.wpi.first.wpilibj.I2C.Port.kOnboard);

	GenericEntry _entryPitch;
	GenericEntry _entryRoll;
	GenericEntry _entryYaw;

	public NavSensor(){
		_nav.calibrate();
		ShuffleboardTab tab = Shuffleboard.getTab(Constants.DATA_TAB);
		_entryPitch = tab.add("Pitch", 0).getEntry();
		_entryRoll = tab.add("Roll", 0).getEntry();
		_entryYaw = tab.add("Yaw", 0).getEntry();
	}

	@Override
	public void periodic() {
		_entryPitch.setDouble(getPitch());
		_entryRoll.setDouble(getRoll());
		_entryYaw.setDouble(getYaw());
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

	double getAccelX() {
		return _nav.getRawAccelX();
	}

	double getAccelY() {
		return _nav.getRawAccelY();
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

	void calibrate() {
		_nav.calibrate();
	}

}
