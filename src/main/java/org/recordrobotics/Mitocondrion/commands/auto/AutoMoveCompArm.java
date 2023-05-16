package org.recordrobotics.Mitocondrion.commands.auto;

import org.recordrobotics.Mitocondrion.commands.manual.ArmPosition;
import org.recordrobotics.Mitocondrion.subsystems.Arm;
import org.recordrobotics.Mitocondrion.subsystems.CompArm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

@SuppressWarnings({"PMD.TooManyFields","PMD.FieldNamingConventions"})
public class AutoMoveCompArm extends CommandBase {
	private CompArm _arm;

	//private static double _second[] = {-122.42, -147.58 - 5/7 * -122.42};
	//private static double _third[] = {-139.79, -139.87 - 5/7 * -139.79};
	private static double _placehold[];

	private static double _second = 46;
	private static double _third = 34;
	//private static double _ground[] = {40, 42};
	private static double _substation = 37.375;

	private static double _neutral[] = {0, 0};
	private static double _secondAngles[] = {0, 0};
	private static double _groundAngles[] = {0, 0};
	private static double _substationAngles[] = {0, 0};
	private static double _thirdAngles[] = {0, 0};

	/*private static double _flipGroundOriginX = 22;
	private static double _flipGroundOriginY = 22;
	private static double _flipGroundChangeX = 22;*/

	public AutoMoveCompArm(CompArm arm) {
		if (arm == null) {
			throw new IllegalArgumentException("Arm is null");
		}

		_arm = arm;
	}

	@Override
	public void execute() {
		// sets arm motor angles based on which actions is needed
		// TODO: Set actual cartesian coords for ALL POSITIONS
		double[] angles = {-84};
		_arm.setAngles(angles);
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