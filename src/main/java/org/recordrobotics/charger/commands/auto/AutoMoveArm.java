package org.recordrobotics.charger.commands.auto;

//import org.recordrobotics.charger.subsystems.Arm2;
import org.recordrobotics.charger.commands.manual.ArmPosition;
import org.recordrobotics.charger.subsystems.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

@SuppressWarnings({"PMD.TooManyFields","PMD.FieldNamingConventions"})
public class AutoMoveArm extends CommandBase {
	private Arm _arm;//was Arm2, changed to make the error go away; this arm code should never be run

	//private PIDController _originPid;
	//private PIDController _changePid;
	//private double _okp = 0.01;
	//private double _oki;
	//private double _okd = 0.005;
	//private double _ckp = 0.01;
	//private double _cki;
	//private double _ckd = 0.005;
	//private double _changeTolerance = 5;
	//private double _originTolerance = 5;
	//private double _maxSpeed = 0.4;
	//private double _maxDownSpeed = 0.2;

	private ArmPosition _armPosition;
	private Timer _timer;

	//private static double _second[] = {-122.42, -147.58 - 5/7 * -122.42};
	//private static double _third[] = {-139.79, -139.87 - 5/7 * -139.79};
	private static double _placehold[];

	private static double _second = 46;
	private static double _third = 34;
	//private static double _ground[] = {40, 42};
	private static double _substation = 37.375;
	private static double _neutral[] = {0, 0};

	/*private static double _flipGroundOriginX = 22;
	private static double _flipGroundOriginY = 22;
	private static double _flipGroundChangeX = 22;*/

	public AutoMoveArm(Arm arm, ArmPosition armPosition) {
		if (arm == null) {
			throw new IllegalArgumentException("Arm is null");
		}

		_arm = arm;
		//_originPid = originPid;
		//_originPid.setD(_okd);
		//_originPid.setI(_oki);
		//_originPid.setP(_okp);
		//_originPid.setTolerance(_originTolerance);
		//_changePid = changePid;
		//_changePid.setD(_ckd);
		//_changePid.setI(_cki);
		//_changePid.setP(_ckp);
		//_changePid.setTolerance(_changeTolerance);
		_armPosition = armPosition;
		_timer = new Timer();
	}

	@Override
	public void execute() {
		// sets arm motor angles based on which actions is needed
		// TODO: Set actual cartesian coords for ALL POSITIONS
		double[] angles;
		switch (_armPosition) {
			case SECOND:
				//angles = _arm.getAnglesRestricted(_second);
				break;
			case THIRD:
				//angles = _arm.getAnglesRestricted(_third);
				break;
			case GROUND:
				angles = _placehold;//_arm.getAnglesOfRotation(_ground[0], _ground[1]);
				break;
			case SUBSTATION:
				//angles = _arm.getAnglesRestricted(_substation);
				break;
			case FLIP_GROUND_ORIGIN:
				angles = _placehold;//_arm.getAnglesOfRotation(_flipGroundOriginX,_flipGroundOriginY);
				break;
			case FLIP_GROUND_CHANGE:
				angles = _placehold;//_arm.getAnglesOfRotation(_flipGroundChangeX, _flipGroundChangeY);
				break;
			default:
				angles = _neutral;
				break;
		}
		//_arm.setAngles(angles);
	}

	public void setArmPosition(ArmPosition position){
		_armPosition = position;
	} 

	@Override
	public void end(boolean interrupted) {
		//placeholder
	}

	@Override
	public boolean isFinished() {
		//return _arm.originAtSetpoint() && _arm.changeAtSetpoint();
		return true;//this was added to make the error go away
	}

}