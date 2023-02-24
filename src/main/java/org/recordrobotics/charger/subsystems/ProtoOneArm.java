// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.recordrobotics.charger.RobotMap;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ProtoOneArm extends SubsystemBase {
	private CANSparkMax _motor = new CANSparkMax(RobotMap.OneMotorArm.ARM_MOTOR_PORT, MotorType.kBrushless);

	private static final double DEGREE_MAX = 90;
	private static final double DEGREE_MIN = 0;
	private static final double GEAR_RATIO = 1/16;

	/** Creates a new OneMotorArm. */
	public ProtoOneArm() {
		_motor.set(0);

		_motor.getEncoder().setPositionConversionFactor(GEAR_RATIO);
	}

	public void turn(double speed) {
		if (_motor.getEncoder().getPosition() * getSign(speed) < DEGREE_MAX && _motor.getEncoder().getPosition() * getSign(speed) > DEGREE_MIN) {
			_motor.set(Subsystems.limitSpeed(speed));
		}
	}

	private int getSign(double val) {
		return val > 0 ? 1 : -1;
	}

	public double getEncoder() {
		return _motor.getEncoder().getPosition();
	}

	public void resetEncoder() {
		_motor.getEncoder().setPosition(0);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}
}
