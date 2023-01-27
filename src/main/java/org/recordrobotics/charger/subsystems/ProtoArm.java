package org.recordrobotics.charger.subsystems;

import org.recordrobotics.charger.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ProtoArm extends Arm {
	private CANSparkMax _originMotor = new CANSparkMax(RobotMap.Arm.ORIGIN_MOTOR_PORT, MotorType.kBrushless);
	private CANSparkMax _changeMotor = new CANSparkMax(RobotMap.Arm.CHANGE_MOTOR_PORT, MotorType.kBrushless);
    
    private static final double GEAR_RATIO = 0;

	public ProtoArm() {
        _originMotor.set(0);
        _changeMotor.set(0);

        _originMotor.getEncoder().setPositionConversionFactor(GEAR_RATIO);
        _changeMotor.getEncoder().setPositionConversionFactor(GEAR_RATIO);
	}

	public void moveAngles(double angle1, double angle2) {
		// move motors
	}
}
