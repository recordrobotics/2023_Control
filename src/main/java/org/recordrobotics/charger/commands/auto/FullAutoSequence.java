package org.recordrobotics.charger.commands.auto;

import java.util.ArrayList;

import org.recordrobotics.charger.commands.manual.ArmPosition;
//import org.recordrobotics.charger.commands.auto.AutoMoveArm;
import org.recordrobotics.charger.subsystems.Drive;
import org.recordrobotics.charger.subsystems.NavSensor;
import org.recordrobotics.charger.subsystems.Vision;
import org.recordrobotics.charger.subsystems.Claw;
import org.recordrobotics.charger.subsystems.Arm;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FullAutoSequence extends SequentialCommandGroup {
	ArmPosition _pos1 = ArmPosition.SECOND;
	ArmPosition _pos2 = ArmPosition.GROUND;
	ArmPosition _pos3 = ArmPosition.THIRD;
	ArmPosition _pos4 = ArmPosition.NEUTRAL;

	private double clawSpeed = 0.05;
	private int clawGrab = 1;
	private int clawRelease = -1;
	private double trajStartTime;

	/**
	 * e
	 */
	public FullAutoSequence(Vision vision, Drive drive, ArrayList<Trajectory> trajectory, DifferentialDrivePoseEstimator estimator, NavSensor nav, Arm arm, Claw claw, double auto_start_time){
		//0 = scoring, 1 = docking, 2 = test
		int sequenceType = 1;

		if (sequenceType == 0){
		addCommands(
			new AutoMoveArm(arm, _pos3),
			new AutoMoveClaw(claw, clawSpeed, clawRelease),
			new VisionDrive(vision, drive, trajectory.get(0), estimator, nav, 0, auto_start_time),//When colored object code gets implemented, use it here
			new AutoMoveArm(arm, _pos2),
			new AutoMoveClaw(claw, clawSpeed, clawGrab),
			new VisionDrive(vision, drive, trajectory.get(1), estimator, nav, 0, auto_start_time),
			new AutoMoveArm(arm, _pos3),
			new AutoMoveClaw(claw, clawSpeed, clawRelease)
		);
		}
		else if (sequenceType == 1){
		addCommands(
			new AutoMoveArm(arm, _pos3),
			new AutoMoveClaw(claw, clawSpeed, clawRelease),
			new VisionDrive(vision, drive, trajectory.get(0), estimator, nav, 0, auto_start_time),
			new ChargeStationBalance(drive, nav)
		);
		}
		else if (sequenceType == 2){
			addCommands(
				new AutoMoveArm(arm, _pos3),
				new AutoMoveArm(arm, _pos2),
				new AutoMoveArm(arm, _pos1),
				new AutoMoveArm(arm, _pos4),
				new VisionDrive(vision, drive, trajectory.get(0), estimator, nav, 0, auto_start_time)
			);
		}
}

}
