package org.recordrobotics.charger.subsystems;

import org.recordrobotics.charger.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
	private WPI_TalonFX _originMotor = new WPI_TalonFX(RobotMap.Arm.ORIGIN_MOTOR_PORT);
	private WPI_TalonFX _changeMotor = new WPI_TalonFX(RobotMap.Arm.CHANGE_MOTOR_PORT);

	private static final FeedbackDevice SELECTED_SENSOR = FeedbackDevice.CTRE_MagEncoder_Relative;

	public Arm() {
		_originMotor.configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 10);
		_changeMotor.configSelectedFeedbackSensor(SELECTED_SENSOR, 0, 10);
	}
}
