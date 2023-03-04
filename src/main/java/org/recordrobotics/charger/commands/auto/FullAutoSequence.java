package org.recordrobotics.charger.commands.auto;

import org.recordrobotics.charger.commands.manual.ArmPosition;
import org.recordrobotics.charger.subsystems.Drive;
import org.recordrobotics.charger.subsystems.NavSensor;
import org.recordrobotics.charger.subsystems.Vision;
import org.recordrobotics.charger.subsystems.Claw;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FullAutoSequence extends SequentialCommandGroup {
	ArmPosition _pos1 = ArmPosition.THIRD;
	ArmPosition _pos2 = ArmPosition.GROUND;
	ArmPosition _pos3 = ArmPosition.THIRD;

	private double clawSpeed = 0.05;
	private int clawGrab = 1;
	private int clawRelease = -1;

	/**
	 * e
	 */
	public FullAutoSequence(Vision vision, Drive drive, Trajectory trajectory, DifferentialDrivePoseEstimator estimator, NavSensor nav, AutoMoveArm mover, Claw claw){
		addCommands(

		new VisionDrive(vision, drive, trajectory, estimator, nav, 0),
		new AutoArmHolder(mover, _pos1),
		new AutoMoveClaw(claw, clawSpeed, clawRelease),
		new VisionDrive(vision, drive, trajectory, estimator, nav, 1),
		new AutoArmHolder(mover, _pos2),
		new AutoMoveClaw(claw, clawSpeed, clawGrab),
		new VisionDrive(vision, drive, trajectory, estimator, nav, 0),
		new AutoArmHolder(mover, _pos3),
		new AutoMoveClaw(claw, clawSpeed, clawRelease)

		);
	}

}
