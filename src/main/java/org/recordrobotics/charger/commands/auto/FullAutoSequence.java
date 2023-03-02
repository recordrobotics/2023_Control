package org.recordrobotics.charger.commands.auto;

import org.recordrobotics.charger.commands.manual.ArmPosition;
import org.recordrobotics.charger.subsystems.Drive;
import org.recordrobotics.charger.subsystems.NavSensor;
import org.recordrobotics.charger.subsystems.Vision;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FullAutoSequence extends SequentialCommandGroup {
	ArmPosition pos1 = ArmPosition.THIRD;
	ArmPosition pos2 = ArmPosition.GROUND;
	ArmPosition pos3 = ArmPosition.THIRD;


	/**
	 * e
	 */
	public FullAutoSequence(Vision vision, Drive drive, Trajectory trajectory, DifferentialDrivePoseEstimator estimator, NavSensor nav, AutoMoveArm mover){
		addCommands(

		new VisionDrive(vision, drive, trajectory, estimator, nav, 0),
		new AutoArmHolder(mover, pos1),
		// do something to drop the cube
		new VisionDrive(vision, drive, trajectory, estimator, nav, 1),
		new AutoArmHolder(mover, pos2),
		// do something to pick up the cube
		new VisionDrive(vision, drive, trajectory, estimator, nav, 0),
		new AutoArmHolder(mover, pos3)
		//do something to drop off the second cube

		);
	}

}
