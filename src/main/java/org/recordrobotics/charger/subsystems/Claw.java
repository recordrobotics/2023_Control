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
<<<<<<< HEAD

	private static final double GEAR_RATIO = 16;

	private CANSparkMax _motor = new CANSparkMax(RobotMap.Claw.MOTOR_PORT, MotorType.kBrushless);

	private GenericEntry _encoderEntry;

	public Claw() {
=======
	private static final double GEAR_RATIO = 63;

	//Open
	public static final double CLAW_NEUTRAL = 0.0;
	//number of turns to grab cube
	public static final double CLAW_CUBE = 0.2;
	//number of turns to grab cone
	public static final double CLAW_CONE = 0.3;

	private CANSparkMax _motor = new CANSparkMax(RobotMap.Claw.MOTOR_PORT, MotorType.kBrushless);

	public Claw(){
>>>>>>> 076f860ed7bc098ed1ce03a391e0afb85b5722bd
		_motor.set(0);
		_motor.getEncoder().setPositionConversionFactor(1 / GEAR_RATIO);

		ShuffleboardTab tab = Shuffleboard.getTab(Constants.constants.DATA_TAB);
		_encoderEntry = tab.add("Drive Encoders", 0).getEntry();
	}

	@Override
	public void periodic() {
		_encoderEntry.setDouble(_motor.getBusVoltage());
	}

<<<<<<< HEAD
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
=======
	public void turn(double speed){
		_motor.set(Subsystems.limitSpeed(speed));
	}

	public double getPosition(){
>>>>>>> 076f860ed7bc098ed1ce03a391e0afb85b5722bd
		return _motor.getEncoder().getPosition();
	}

	/**
	 * resets encoder values
	 */
	public void resetEncoders() {
		_motor.getEncoder().setPosition(0);
	}
}
