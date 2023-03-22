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

	/**
	 * e
	 */
	public FullAutoSequence(Vision vision, Drive drive, ArrayList<Trajectory> trajectory, DifferentialDrivePoseEstimator estimator, NavSensor nav, AutoMoveArm mover, Claw claw, Arm arm){
		String sequenceType = "test";

		if (sequenceType == "scoring"){
		addCommands(
			new AutoMoveArm(arm, _pos3),
			new AutoMoveClaw(claw, clawSpeed, clawRelease),
			new VisionDrive(vision, drive, trajectory.get(0), estimator, nav, 0),//When colored object code gets implemented, use it here
			new AutoMoveArm(arm, _pos2),
			new AutoMoveClaw(claw, clawSpeed, clawGrab),
			new VisionDrive(vision, drive, trajectory.get(1), estimator, nav, 0),
			new AutoMoveArm(arm, _pos3),
			new AutoMoveClaw(claw, clawSpeed, clawRelease)
		);
		}
		else if (sequenceType == "docking"){
		addCommands(
			new AutoMoveArm(arm, _pos3),
			new AutoMoveClaw(claw, clawSpeed, clawRelease),
			new VisionDrive(vision, drive, trajectory.get(0), estimator, nav, 0),
			new ChargeStationBalance(drive, nav)
		);
		}
		else if (sequenceType == "test"){
			addCommands(
				new AutoMoveArm(arm, _pos3),
				new AutoMoveArm(arm, _pos2),
				new AutoMoveArm(arm, _pos1),
				new AutoMoveArm(arm, _pos4)
				//new VisionDrive(vision, drive, trajectory.get(0), estimator, nav, 0)
			);
		}
}

}
