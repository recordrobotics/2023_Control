package org.recordrobotics.charger.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.recordrobotics.charger.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Claw extends SubsystemBase {

	private static final double GEAR_RATIO = 63;

	private CANSparkMax _motor = new CANSparkMax(RobotMap.Claw.MOTOR_PORT, MotorType.kBrushless);

	public Claw() {
		_motor.set(0);
		_motor.getEncoder().setPositionConversionFactor(1 / GEAR_RATIO);
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
