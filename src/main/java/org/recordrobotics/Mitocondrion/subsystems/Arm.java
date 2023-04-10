package org.recordrobotics.Mitocondrion.subsystems;

import org.recordrobotics.Mitocondrion.Constants;
import org.recordrobotics.Mitocondrion.RobotMap;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase{
    private WPI_TalonFX _originMotor = new WPI_TalonFX(RobotMap.Arm.ORIGIN_MOTOR_PORT);
	private WPI_TalonFX _changeMotor = new WPI_TalonFX(RobotMap.Arm.CHANGE_MOTOR_PORT);
	public static final double FIRST_ARM_LENGTH = Units.inchesToMeters(38); //TODO: All of these lengths and angles are off slightly, and should be modified
	public static final double SECOND_ARM_LENGTH = Units.inchesToMeters(28); //31 when claw is closed 
	private static final double ARM_BASE_HEIGHT = 14.75;
    //private static final double FIRST_ARM_ZERO = Math.PI/3;
    //private static final double SECOND_ARM_ZERO = Math.PI;
	private static final double TICKS_PER_REV = 2048;
	private static final double GEAR_RATIO = 48;
	// private static final double ERROR_MARGIN = 0;

	private PIDController _originPid;
	private PIDController _changePid;
	private static final double C_KP = 0.025;
	private static final double C_KI = 0.005;
	private static final double C_KD = 0;
	private static final double O_KP = 0.025;
	private static final double O_KI = 0.005;
	private static final double O_KD = 0;
	private double _changeTolerance = 0.5;
	private double _originTolerance = 0.5;
	private double _originMaxSpeed = 0.5;
	private double _changeMaxSpeed = 0.5;
	private double[] prevAngles = {0, 0};

	private double[] commandAngles = {0, 0};
	private double rampConstant = 1;

	private GenericEntry _entryAngles;

	private static final TalonFXConfiguration armConfig = new TalonFXConfiguration();

	private double[] _angles = new double[2];
    
    public Arm() {
		armConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
		armConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		_originMotor.configAllSettings(armConfig);
		_changeMotor.configAllSettings(armConfig);
		_originMotor.setNeutralMode(NeutralMode.Brake);
		_changeMotor.setNeutralMode(NeutralMode.Brake);
		_originMotor.set(0);
		_changeMotor.set(0);

		_originPid = new PIDController(O_KP, O_KI, O_KD);
		_originPid.setTolerance(_originTolerance);
		_changePid = new PIDController(C_KP, C_KI, C_KD);
		_changePid.setTolerance(_changeTolerance);

		resetEncoders();

		ShuffleboardTab tab = Shuffleboard.getTab(Constants.DATA_TAB);
		_entryAngles = tab.add("Pure sensor output", new double[] {0, 0}).getEntry();
	}

	public void setAngles(double[] angles) {
		commandAngles[0] = angles[0];
		commandAngles[1] = angles[1];
	}

	public double[] getCurrentAngles() {
		return _angles;
	}

    public double[] getAngles(double x, double y, String direction) {//For more information, see Modern Robotics: Mechanics, Design and Control, chapter 6
		double L1 = FIRST_ARM_LENGTH;
		double L2 = SECOND_ARM_LENGTH;
        double beta = Math.acos((L1*L1 + L2*L2 - x*x - y*y)/(2*L1*L2));
        double alpha = Math.acos((x*x + y*y + L1*L1 - L2*L2)/(2*L1*Math.sqrt(x*x + y*y)));
        double gamma = Math.atan2(y, x);
        double theta1 = 0;
        double theta2 = 0;
        if (direction.equals("R")){
            theta1 = gamma - alpha;
            theta2 = Math.PI - beta;
        }
        else if(direction.equals("L")){
            theta1 = gamma + alpha;
            theta2 = beta - Math.PI;
        }
        else{
            System.out.println("invalid direction set");
        }
        double[] angles = {theta1, theta2};
        return angles;
    }

    public double[] getAnglesRestricted(double y){//Angles but holding the second segment parallel to the ground
		double L1 = FIRST_ARM_LENGTH;
        double theta1 = Math.asin(y/L1);
        double theta2 = Math.PI - theta1;
        double[] angles = {theta1, theta2};
        return angles;
    }

    public void spinOrigin(double speed) {
		_originMotor.set(speed);
	}

	public void spinChange(double speed) {
		_changeMotor.set(speed);
	}

	public boolean originAtSetpoint() {
		return _originPid.atSetpoint();
	}

	public boolean changeAtSetpoint() {
		return _changePid.atSetpoint();
	}
	
    //public double[] getCurrentAngles(){ // Why does this exist?
        //double[] currentAngles = {getOriginEncoder() + FIRST_ARM_ZERO, getChangeEncoder() + SECOND_ARM_ZERO};
        //return currentAngles;
    //}

    public double getOriginEncoderSpeed(){
        return _originMotor.getSelectedSensorVelocity() / TICKS_PER_REV * Math.PI / GEAR_RATIO;//TODO: TUNE THE CONFIGS FOR THESE
    }

    public double getChangeEncoderSpeed(){
        return _changeMotor.getSelectedSensorVelocity() / TICKS_PER_REV * Math.PI / GEAR_RATIO;
    }

    /**
	 * @return value of origin motor encoder in RADIANS
	 */
	public double getOriginEncoder() {
		return _originMotor.getSelectedSensorPosition() / TICKS_PER_REV * 360 / GEAR_RATIO;
	}

	/**
	 * @return value of change motor encoder in RADIANS
	 */
	public double getChangeEncoder() {
		return _changeMotor.getSelectedSensorPosition() / TICKS_PER_REV * 360 / GEAR_RATIO;
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
		_changePid = new PIDController(C_KP, C_KI, C_KD);
		_changePid.setTolerance(_changeTolerance);
	}

	@Override
	public void periodic() {

		if(_angles[0] > commandAngles[0]) {
			_angles[0] -= rampConstant;
		} else if(_angles[0] < commandAngles[0]) {
			_angles[0] += rampConstant;
		}
		if(_angles[1] > commandAngles[1]) {
			_angles[1] -= rampConstant;
		} else if(_angles[1] < commandAngles[1]) {
			_angles[1] += rampConstant;
		}

		SmartDashboard.putNumber("Raw Origin Encoder", _originMotor.getSelectedSensorPosition());
		SmartDashboard.putNumber("Raw Change Encoder", _changeMotor.getSelectedSensorPosition());
		_entryAngles.setDoubleArray(_angles);

//		if(_angles[0] != prevAngles[0]) {
    		_originPid.setSetpoint(_angles[0]);
			SmartDashboard.putNumber("Origin Target", _angles[0]);
			prevAngles[0] = _angles[0];
//		}	
//		if(_angles[1] != prevAngles[1]) {
			_changePid.setSetpoint(_angles[1]);
			SmartDashboard.putNumber("Change Target", _angles[1]);
			prevAngles[1] = _angles[1];
//		}

		double originPos = getOriginEncoder();
		double changePos = getChangeEncoder();
		SmartDashboard.putNumber("Origin Pos", originPos);
		SmartDashboard.putNumber("Change Pos", changePos);
		double _originSpeed = _originPid.calculate(originPos);
		double _changeSpeed = _changePid.calculate(changePos);  
		_originSpeed = Math.min(Math.abs(_originSpeed), _originMaxSpeed) * Math.signum(_originSpeed);
		_changeSpeed = Math.min(Math.abs(_changeSpeed), _changeMaxSpeed) * Math.signum(_changeSpeed);
		SmartDashboard.putNumber("Origin Speed", _originSpeed);
		SmartDashboard.putNumber("Change Speed", _changeSpeed);
		
		spinOrigin(_originSpeed);
		spinChange(_changeSpeed);
	}
}
