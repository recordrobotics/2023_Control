package org.recordrobotics.Mitocondrion.commands.auto;

import java.util.ArrayList;
import java.util.function.BiConsumer;

import org.recordrobotics.Mitocondrion.commands.manual.ArmPosition;
import org.recordrobotics.Mitocondrion.subsystems.Drive;
import org.recordrobotics.Mitocondrion.subsystems.NavSensor;
import org.recordrobotics.Mitocondrion.subsystems.Vision;
import org.recordrobotics.Mitocondrion.subsystems.Claw;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FullAuto extends SequentialCommandGroup {
	ArmPosition _pos1 = ArmPosition.THIRD;
	ArmPosition _pos2 = ArmPosition.GROUND;
	ArmPosition _pos3 = ArmPosition.THIRD;

	private double clawSpeed = 0.05;
	private int clawGrab = 1;
	private int clawRelease = -1;

	/**
	 * e
	 */
	public FullAuto(Vision vision, Drive drive, ArrayList<Trajectory> trajectory, RamseteController ramsete, DifferentialDriveKinematics kinematics, DifferentialDrivePoseEstimator estimator, NavSensor nav, AutoMoveArm mover, Claw claw){
		String sequenceType = "test";

		if (sequenceType == "scoring"){
		addCommands(
			new AutoArmHolder(mover, _pos1),
			new AutoMoveClaw(claw, clawSpeed, clawRelease),
			new VisionDrive(vision, drive, trajectory.get(0), estimator, nav, 0),//When colored object code gets implemented, use it here
			new AutoArmHolder(mover, _pos2),
			new AutoMoveClaw(claw, clawSpeed, clawGrab),
			new VisionDrive(vision, drive, trajectory.get(1), estimator, nav, 0),
			new AutoArmHolder(mover, _pos3),
			new AutoMoveClaw(claw, clawSpeed, clawRelease)
		);
		}
		else if (sequenceType == "docking"){
		addCommands(
			new AutoArmHolder(mover, _pos1),
			new AutoMoveClaw(claw, clawSpeed, clawRelease),
			new VisionDrive(vision, drive, trajectory.get(0), estimator, nav, 0),
			new ChargeStationBalance(drive, nav)
		);
		}
		else if (sequenceType == "test"){
			addCommands(
                new RamseteCommand(trajectory.get(0), estimator::getEstimatedPosition, ramsete, 
                new SimpleMotorFeedforward(0, 0), kinematics, drive::getWheelSpeeds, new PIDController(0, 0, 0), 
                new PIDController(0, 0, 0), drive::tankDriveVolts, drive, nav, vision)
			);
		}
}

}