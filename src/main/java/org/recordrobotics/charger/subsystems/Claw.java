package org.recordrobotics.charger.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.recordrobotics.charger.Constants;
import org.recordrobotics.charger.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Claw extends SubsystemBase {

	private static final double GEAR_RATIO = 80;

	private CANSparkMax _motor = new CANSparkMax(RobotMap.Claw.MOTOR_PORT, MotorType.kBrushless);
	private DigitalInput _limitSwitch = new DigitalInput(RobotMap.Claw.LIMIT_SWITCH);

	private GenericEntry _voltageEntry;
	private GenericEntry _encoderEntry;

	public Claw() {
		_motor.set(0);
		_motor.getEncoder().setPositionConversionFactor(1 / GEAR_RATIO);

		ShuffleboardTab tab = Shuffleboard.getTab(Constants.Tab.DATA_TAB);
		_voltageEntry = tab.add("Motor Current", 0).getEntry();
		_encoderEntry = tab.add("Encoder Value", 0).getEntry();
	}

	@Override
	public void periodic() {
		_voltageEntry.setDouble(_motor.getOutputCurrent());
		_encoderEntry.setDouble(getPosition());
	}

	/*
	 * Returns the the Claw Switch
	 *  @return true if Claw in boundary
	 */
	public boolean getSwitchState(){
		return !_limitSwitch.get();
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
