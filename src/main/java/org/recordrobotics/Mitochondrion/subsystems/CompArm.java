package org.recordrobotics.Mitochondrion.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.recordrobotics.Mitochondrion.Constants;
import org.recordrobotics.Mitochondrion.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import org.recordrobotics.Mitochondrion.Constants;
import org.recordrobotics.Mitochondrion.RobotMap;

public class CompArm extends SubsystemBase{
    private WPI_TalonFX _originMotor = new WPI_TalonFX(RobotMap.Arm.ORIGIN_MOTOR_PORT);
	private static final double TICKS_PER_REV = 2048;
	private static final double GEAR_RATIO = 48;
	// private static final double ERROR_MARGIN = 0;

	private PIDController _originPid;
	private static final double O_KP = 0.025;
	private static final double O_KI = 0.005;
	private static final double O_KD = 0;
	private double _originTolerance = 0.5;
	private double _originMaxSpeed = 0.5;
	private double[] prevAngles = {0, 0};

	private double[] commandAngles = {0};
	private double rampConstant = 1;

	private GenericEntry _entryAngles;

	private static final TalonFXConfiguration armConfig = new TalonFXConfiguration();

	private double[] _angles = new double[1];
    
    public CompArm() {
		armConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
		armConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		_originMotor.configAllSettings(armConfig);
		_originMotor.setNeutralMode(NeutralMode.Brake);
		_originMotor.set(0);

		_originPid = new PIDController(O_KP, O_KI, O_KD);
		_originPid.setTolerance(_originTolerance);

		resetEncoders();

		ShuffleboardTab tab = Shuffleboard.getTab(Constants.DATA_TAB);
		_entryAngles = tab.add("Pure sensor output", new double[] {0, 0}).getEntry();
	}

    public void setAngles(double[] angles) {
		commandAngles[0] = angles[0];
	}

	/**
	 * resets origin and change motor encoders
	 */
	public void resetEncoders() {
		_originMotor.setSelectedSensorPosition(0);
	}

    public void resetPID() {
		_originPid = new PIDController(O_KP, O_KI, O_KD);
		_originPid.setTolerance(_originTolerance);
	}

    public void spinOrigin(double speed) {
		_originMotor.set(speed);
	}

    /**
	 * @return value of origin motor encoder in degrees
	 */
	public double getOriginEncoder() {
		return _originMotor.getSelectedSensorPosition() / TICKS_PER_REV * 360 / GEAR_RATIO;
	}

    @Override
	public void periodic() {
		if(_angles[0] > commandAngles[0]) {
			_angles[0] -= rampConstant;
		} else if(_angles[0] < commandAngles[0]) {
			_angles[0] += rampConstant;
		}
		
		SmartDashboard.putNumber("Raw Origin Encoder", _originMotor.getSelectedSensorPosition());
		_entryAngles.setDoubleArray(_angles);

//		if(_angles[0] != prevAngles[0]) {
    		_originPid.setSetpoint(_angles[0]);
			SmartDashboard.putNumber("Origin Target", _angles[0]);
			prevAngles[0] = _angles[0];
//		}	

		double originPos = getOriginEncoder();
		SmartDashboard.putNumber("Origin Pos", originPos);
		double _originSpeed = _originPid.calculate(originPos);
		_originSpeed = Math.min(Math.abs(_originSpeed), _originMaxSpeed) * Math.signum(_originSpeed);
		SmartDashboard.putNumber("Origin Speed", _originSpeed);
		
		spinOrigin(_originSpeed);
	}
	public boolean originAtSetpoint() {
		return _originPid.atSetpoint();
	}
}
