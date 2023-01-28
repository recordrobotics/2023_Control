package org.recordrobotics.charger.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.recordrobotics.charger.RobotMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Claw extends SubsystemBase {
	private CANSparkMax _motor = new CANSparkMax(RobotMap.Claw.CLAW_MOTOR_PORT, MotorType.kBrushless);

	public RelativeEncoder _motorEncoder = _motor.getEncoder();

	public Claw(){
		_motor.set(0);
	}


	public void turn(double speed){
		if(speed > 0 && _motorEncoder.getPosition() < 0.5 || speed < 0 && _motorEncoder.getPosition() > 0){
			_motor.set(Subsystems.limitSpeed(speed));
		}else{
			_motor.set(0);
		}
	}

	public double getPosition(){
		return _motorEncoder.getPosition()/63;
	}

	public void resetEncoders() {
		_motorEncoder.setPosition(0);
	}
}
