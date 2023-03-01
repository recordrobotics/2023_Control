package org.recordrobotics.charger.commands.auto;

import org.recordrobotics.charger.subsystems.Drive;
import org.recordrobotics.charger.subsystems.NavSensor;
import org.recordrobotics.charger.subsystems.Vision;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FullAutoSequence extends SequentialCommandGroup {

	/**
	 * e
	 */
	public FullAutoSequence(Vision vision, Drive drive, Trajectory trajectory, DifferentialDrivePoseEstimator estimator, NavSensor nav){
		addCommands(

		new VisionDrive(vision, drive, trajectory, estimator, nav, 0),
		// do something to drop the cube
		new VisionDrive(vision, drive, trajectory, estimator, nav, 1),
		// do something to pick up the cube
		new VisionDrive(vision, drive, trajectory, estimator, nav, 0)
		//do something to drop off the second cube

		);
	}

}
