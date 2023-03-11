package org.recordrobotics.charger.subsystems;

import org.recordrobotics.charger.Constants;
import org.recordrobotics.charger.RobotMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm2 extends SubsystemBase{
    private WPI_TalonFX _originMotor = new WPI_TalonFX(RobotMap.Arm.ORIGIN_MOTOR_PORT);
	private WPI_TalonFX _changeMotor = new WPI_TalonFX(RobotMap.Arm.CHANGE_MOTOR_PORT);
	public static final double FIRST_ARM_LENGTH = Units.inchesToMeters(40);//TODO: All of these lengths and angles are off slightly, and should be modified
	public static final double SECOND_ARM_LENGTH = Units.inchesToMeters(20);//TODO: check all of these constants
    private static final double FIRST_ARM_ZERO = Math.PI/3;
    private static final double SECOND_ARM_ZERO = Math.PI;
	private static final double TICKS_PER_REV = 2048;
	private static final double GEAR_RATIO = 1;//TODO: This should be changed to 16
	private static final double ERROR_MARGIN = 0;

	private GenericEntry _entryAngles;

	private static final TalonFXConfiguration armConfig = new TalonFXConfiguration();

	private double[] _angles = new double[2];
    
    public Arm2() {
		armConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;
		_originMotor.configAllSettings(armConfig);
		_changeMotor.configAllSettings(armConfig);
		_originMotor.setNeutralMode(NeutralMode.Brake);
		_changeMotor.setNeutralMode(NeutralMode.Brake);
		_originMotor.set(0);
		_changeMotor.set(0);

		resetEncoders();

		ShuffleboardTab tab = Shuffleboard.getTab(Constants.DATA_TAB);
		_entryAngles = tab.add("Angles Of Rotation", new double[] {0, 0}).getEntry();
	}


    public double[] getAngles(double L1, double L2, double x, double y, String direction) {//For more information, see Modern Robotics: Mechanics, Design and Control, chapter 6
        double beta = Math.acos((L1*L1 + L2*L2 - x*x - y*y)/(2*L1*L2));
        double alpha = Math.acos((x*x + y*y + L1*L1 - L2*L2)/(2*L1*Math.sqrt(x*x + y*y)));
        double gamma = Math.atan2(y, x);
        double theta1 = 0;
        double theta2 = 0;
        if (direction == "R"){
            theta1 = gamma - alpha;
            theta2 = Math.PI - beta;
        }
        else if(direction == "L"){
            theta1 = gamma + alpha;
            theta2 = beta - Math.PI;
        }
        else{
            System.out.println("invalid direction set");
        }
        double[] angles = {theta1, theta2};
        return angles;
    }

    public double[] getAnglesRestricted(double L1, double L2, double y){//Angles but holding the second segment parallel to the ground
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

    public double[] getCurrentAngles(){
        double[] currentAngles = {getOriginEncoder() + FIRST_ARM_ZERO, getChangeEncoder() + SECOND_ARM_ZERO};
        return currentAngles;
    }

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
		System.out.println("Origin Encoder Raw = " + _originMotor.getSelectedSensorPosition());
		return (_originMotor.getSelectedSensorPosition() / TICKS_PER_REV * 2 * Math.PI / GEAR_RATIO);
	}

	/**
	 * @return value of change motor encoder in RADIANS
	 */
	public double getChangeEncoder() {
		System.out.println("Change Encoder Raw = " + _changeMotor.getSelectedSensorPosition());
		return _changeMotor.getSelectedSensorPosition() / TICKS_PER_REV * 2 * Math.PI / GEAR_RATIO;
	}

	/**
	 * resets origin and change motor encoders
	 */
	public void resetEncoders() {
		_originMotor.setSelectedSensorPosition(0);
		_changeMotor.setSelectedSensorPosition(0);
	}

	@Override
	public void periodic() {
		_entryAngles.setDoubleArray(_angles);
	}
}
