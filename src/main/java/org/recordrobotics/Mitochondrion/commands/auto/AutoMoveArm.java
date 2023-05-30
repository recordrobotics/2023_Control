package org.recordrobotics.Mitochondrion.commands.auto;

import org.recordrobotics.Mitochondrion.commands.manual.ArmPosition;
import org.recordrobotics.Mitochondrion.subsystems.Arm;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

@SuppressWarnings({"PMD.TooManyFields","PMD.FieldNamingConventions"})
public class AutoMoveArm extends CommandBase {
	private Arm _arm;

	//private static double _second[] = {-122.42, -147.58 - 5/7 * -122.42};
	//private static double _third[] = {-139.79, -139.87 - 5/7 * -139.79};
	private static double _placehold[];

	private static double _second = 46;
	private static double _third = 34;
	//private static double _ground[] = {40, 42};
	private static double _substation = 37.375;
	private double[] pos = {0, 0};
	private double[] angles = {0, 0};
	
	public final static double maX = Units.inchesToMeters(21);
	public final static double maxY = Units.inchesToMeters(75) - Arm.ARM_BASE_HEIGHT;
	public final static double minY = 0.25;
	public final static double minX = -0.5;

	private static double _neutral[] = {0, 0};
	private static double _secondAngles[] = {0, 0};
	private static double _groundAngles[] = {0, 0};
	private static double _substationAngles[] = {0, 0};
	private static double _thirdAngles[] = {0, 0};

	/*private static double _flipGroundOriginX = 22;
	private static double _flipGroundOriginY = 22;
	private static double _flipGroundChangeX = 22;*/

	public AutoMoveArm(Arm arm) {
		if (arm == null) {
			throw new IllegalArgumentException("Arm is null");
		}

		_arm = arm;
	}

	@Override
	public void execute() {
		pos[0] = Units.inchesToMeters(34.625);
		pos[1] = Units.inchesToMeters(38.5) - Arm.ARM_BASE_HEIGHT;

		pos[0] = Math.max(Math.min(pos[0], maX), minX);
		pos[1] = Math.max(Math.min(pos[1], maxY), minY);
		angles = _arm.getAngles(pos[0], pos[1], "L");

		SmartDashboard.putNumber("Pos X", pos[0]);
		SmartDashboard.putNumber("Pos Y", pos[1]);
		SmartDashboard.putNumber("command set origin", angles[0]);
		_arm.setAngles(angles);// sets arm motor angles based on which actions is needed
	}

	public void setArmPosition(ArmPosition position){
		//_armPosition = position;
	}

	@Override
	public void end(boolean interrupted) {
		//placeholder
	}

	@Override
	public boolean isFinished() {
		return _arm.originAtSetpoint();
	}

}