package org.recordrobotics.charger.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.recordrobotics.charger.Constants;
import org.recordrobotics.charger.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Claw extends SubsystemBase {

	private static final double GEAR_RATIO = 16;

	private CANSparkMax _motor = new CANSparkMax(RobotMap.Claw.MOTOR_PORT, MotorType.kBrushless);

	private GenericEntry _voltageEntry;
	private GenericEntry _encoderEntry;

	public Claw() {
		_motor.set(0);
		_motor.getEncoder().setPositionConversionFactor(1 / GEAR_RATIO);

		ShuffleboardTab tab = Shuffleboard.getTab(Constants.constants.DATA_TAB);
		_voltageEntry = tab.add("Voltage", 0).getEntry();
		_encoderEntry = tab.add("Drive Encoders", 0).getEntry();
	}

	@Override
	public void periodic() {
		_voltageEntry.setDouble(_motor.getBusVoltage());
		_encoderEntry.setDouble(getPosition());
	}

	/**
	 * turns motor
	 * @param speed speed of the motor
	 */
	public void turn(double speed) {
		_motor.set(Subsystems.limitSpeed(speed));
	}

	/**
	 * gets encoder value of
	 * @return
	 */
	public double getPosition() {
		return _motor.getEncoder().getPosition();
	}

	/**
	 * resets encoder values
	 */
	public void resetEncoders() {
		_motor.getEncoder().setPosition(0);
	}

}
