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

	private static final double GEAR_RATIO = 125;

	private CANSparkMax _motor = new CANSparkMax(RobotMap.Claw.MOTOR_PORT, MotorType.kBrushless);
	private DigitalInput _limitSwitch = new DigitalInput(RobotMap.Claw.LIMIT_SWITCH);

	private GenericEntry _currentEntry;
	private GenericEntry _encoderEntry;
	private GenericEntry _switchEntry;

	public double OPEN_CLAW_ENCODER = 0.2;
	public double CURRENT_GRAB_THRESHOLD = 5;

	public Claw() {
		_motor.set(0);
		_motor.getEncoder().setPositionConversionFactor(1 / GEAR_RATIO);

		ShuffleboardTab tab = Shuffleboard.getTab(Constants.DATA_TAB);
		_currentEntry = tab.add("Motor Current", 0).getEntry();
		_encoderEntry = tab.add("Encoder Value", 0).getEntry();
		_switchEntry = tab.add("Limit Switch", false).getEntry();
	}

	@Override
	public void periodic() {
		_currentEntry.setDouble(_motor.getOutputCurrent());
		_encoderEntry.setDouble(getPosition());
		_switchEntry.setBoolean(getSwitchState());
	}

	/**
	 * Check if switch is pressed
	 * @return True if switch is triggered
	 */
	public boolean getSwitchState(){
		return !_limitSwitch.get();
	}

	/**
	 * Run motor
	 * @param speed Speed
	 */
	public void turn(double speed) {
		_motor.set(Subsystems.limitSpeed(speed));
	}

	/**
	 * Get encoder value of the claw
	 * @return The encoder value
	 */
	public double getPosition() {
		return _motor.getEncoder().getPosition();
	}

	/**
	 * Get motor current
	 * @return Motor current
	 */
	public double getCurrent() {
		return _motor.getOutputCurrent();
	}

	/**
	 * Reset encoder value
	 */
	public void resetEncoders() {
		_motor.getEncoder().setPosition(0);
	}

}
