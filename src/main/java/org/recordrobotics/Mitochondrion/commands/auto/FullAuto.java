package org.recordrobotics.Mitochondrion.commands.auto;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.recordrobotics.Mitochondrion.commands.manual.ArmPosition;
import org.recordrobotics.Mitochondrion.subsystems.Arm;
import org.recordrobotics.Mitochondrion.subsystems.Claw;
import org.recordrobotics.Mitochondrion.subsystems.Drive;
import org.recordrobotics.Mitochondrion.subsystems.NavSensor;
import org.recordrobotics.Mitochondrion.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FullAuto extends SequentialCommandGroup {
	ArmPosition _pos1 = ArmPosition.THIRD;
	ArmPosition _pos2 = ArmPosition.GROUND;
	ArmPosition _pos3 = ArmPosition.THIRD;

	private double _kp = 1;
	private double clawSpeed = 0.3;
	private int clawGrab = 1;
	private int clawRelease = -1;
	private PhotonCamera _cam;

	/**
	 * e
	 */
	public FullAuto(Vision vision, Drive drive, ArrayList<Trajectory> trajectory, RamseteController ramsete, DifferentialDriveKinematics kinematics, DifferentialDrivePoseEstimator estimator, NavSensor nav, Arm arm, Claw claw){
		String sequenceType = "scorething";

		if (sequenceType == "scoring"){
		addCommands(
			//new AutoArmHolder(mover, _pos1),
			new AutoMoveClaw(claw, clawSpeed, clawRelease),
			new VisionDrive(vision, drive, trajectory.get(0), estimator, nav, 0),//When colored object code gets implemented, use it here
			//new AutoArmHolder(mover, _pos2),
			new AutoMoveClaw(claw, clawSpeed, clawGrab),
			new VisionDrive(vision, drive, trajectory.get(1), estimator, nav, 0),
			//new AutoArmHolder(mover, _pos3),
			new AutoMoveClaw(claw, clawSpeed, clawRelease)
		);
		}
		else if (sequenceType == "docking"){
		addCommands(
			//new AutoArmHolder(mover, _pos1),
			new AutoMoveClaw(claw, clawSpeed, clawRelease),
			new VisionDrive(vision, drive, trajectory.get(0), estimator, nav, 0),
			new ChargeStationBalance(drive, nav)
		);
		}
		else if (sequenceType == "test"){
			addCommands(
                new RamseteCommand(trajectory.get(0), estimator::getEstimatedPosition, ramsete, 
                new SimpleMotorFeedforward(-0.12215, 1.4629, 5.9068), kinematics, drive::getWheelSpeeds, new PIDController(_kp, 0, 0), 
                new PIDController(_kp, 0, 0), drive::tankDriveVolts, drive, nav, vision)
			);
		}
		else if (sequenceType == "scorething") {
			addCommands(
				// Simple scoring
				new AutoMoveArm(arm, ArmPosition.THIRD),
				new WaitCommand(2),
				new AutoMoveClaw(claw, clawSpeed, -1),
				new AutoMoveArm(arm, ArmPosition.NEUTRAL)
			);
		}


		else if (sequenceType == "dontcrash") {

			// 15.513558 - 2.820162 = 12.693396 (RED  = side 1)
			// 1.02743 + 2.820162 = 3.847592 (BLUE = side 0)

			double [] targetPose = {12.693396, 1.071626, Math.PI}; //CHANGE THE Y VALUE
			double side = 1; // FOR SIDE: 0 = blue, 1 = red
			
			_cam = vision.camera;
			addCommands(
				// Simple scoring
				new AutoMoveArm(arm, ArmPosition.THIRD),
				new AutoMoveClaw(claw, clawSpeed, -1),
				new AutoMoveArm(arm, ArmPosition.NEUTRAL),

				// Moves/taxi
				new VisionMoveToPoint(vision, drive, targetPose, _cam, side),

				// Balances
				new ChargeStationBalance(drive, nav)
			);
		}
}

}