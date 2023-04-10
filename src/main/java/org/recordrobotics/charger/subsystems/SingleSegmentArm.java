package org.recordrobotics.charger.subsystems;

import org.recordrobotics.charger.Constants;
import org.recordrobotics.charger.RobotMap;

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


public class SingleSegmentArm extends SubsystemBase{
	private WPI_TalonFX _motor = new WPI_TalonFX(RobotMap.Arm.CHANGE_MOTOR_PORT);
	public static final double ARM_LENGTH = Units.inchesToMeters(28); //31 when claw is closed 
	private static final double ARM_BASE_HEIGHT = 14.75;
	private static final double TICKS_PER_REV = 2048;
	private static final double GEAR_RATIO = 48;
	// private static final double ERROR_MARGIN = 0;

    private PIDController _changePid;
	private static final double C_KP = 0.025;
	private static final double C_KI = 0.005;
	private static final double C_KD = 0;
	private double _tolerance = 0.5;
	private double _maxSpeed = 0.5;
	private double[] prevAngles = {0, 0};

	private double[] commandAngles = {0, 0};
	private double rampConstant = 1;

	private GenericEntry _entryAngles;

	private static final TalonFXConfiguration armConfig = new TalonFXConfiguration();

	private double[] _angles = new double[2];
    
    public SingleSegmentArm() {
		armConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
		armConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
		_motor.configAllSettings(armConfig);
		_motor.setNeutralMode(NeutralMode.Brake);
		_motor.set(0);

		_changePid = new PIDController(C_KP, C_KI, C_KD);
		_changePid.setTolerance(_tolerance);

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

    /*
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

    */
    public void spinMotor(double speed) {
		_motor.set(speed);
	}

	public boolean changeAtSetpoint() {
		return _changePid.atSetpoint();
	}
	
    //public double[] getCurrentAngles(){ // Why does this exist?
        //double[] currentAngles = {getOriginEncoder() + FIRST_ARM_ZERO, getChangeEncoder() + SECOND_ARM_ZERO};
        //return currentAngles;
    //}

    public double getChangeEncoderSpeed(){
        return _motor.getSelectedSensorVelocity() / TICKS_PER_REV * Math.PI / GEAR_RATIO;
    }

	/**
	 * @return value of change motor encoder in RADIANS
	 */
	public double getChangeEncoder() {
		return _motor.getSelectedSensorPosition() / TICKS_PER_REV * 360 / GEAR_RATIO;
	}

	/**
	 * resets origin and change motor encoders
	 */
	public void resetEncoders() {
		_motor.setSelectedSensorPosition(0);
	}

	public void resetPID() {

		_changePid = new PIDController(C_KP, C_KI, C_KD);
		_changePid.setTolerance(_tolerance);
	}

	@Override
	public void periodic() {

		/*if(_angles[0] > commandAngles[0]) {
			_angles[0] -= rampConstant;
		} else if(_angles[0] < commandAngles[0]) {
			_angles[0] += rampConstant;
		}*/
		if(_angles[1] > commandAngles[1]) {
			_angles[1] -= rampConstant;
		} else if(_angles[1] < commandAngles[1]) {
			_angles[1] += rampConstant;
		}

		SmartDashboard.putNumber("Raw Change Encoder", _motor.getSelectedSensorPosition());
		_entryAngles.setDoubleArray(_angles);

//		}	
//		if(_angles[1] != prevAngles[1]) {
			_changePid.setSetpoint(_angles[1]);
			SmartDashboard.putNumber("Change Target", _angles[1]);
			prevAngles[1] = _angles[1];
//		}

		double changePos = getChangeEncoder();
		SmartDashboard.putNumber("Change Pos", changePos);
		double _changeSpeed = _changePid.calculate(changePos); 
		
		_changeSpeed = Math.min(Math.abs(_changeSpeed), _maxSpeed) * Math.signum(_changeSpeed);
		SmartDashboard.putNumber("Change Speed", _changeSpeed);
		spinMotor(_changeSpeed);
	}
}
