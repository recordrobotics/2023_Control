package org.recordrobotics.charger.commands.auto;

import java.util.ArrayList;

import org.recordrobotics.charger.commands.manual.ArmPosition;
import org.recordrobotics.charger.subsystems.Drive;
import org.recordrobotics.charger.subsystems.NavSensor;
import org.recordrobotics.charger.subsystems.Vision;
import org.recordrobotics.charger.subsystems.Arm;
import org.recordrobotics.charger.subsystems.Claw;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class TestPreset extends SequentialCommandGroup {
	ArmPosition _pos1 = ArmPosition.THIRD;
	ArmPosition _pos2 = ArmPosition.GROUND;
	ArmPosition _pos3 = ArmPosition.THIRD;

	private double clawSpeed = 0.05;
	private int clawGrab = 1;
	private int clawRelease = -1;
	/**
	 * e
	 */
	public TestPreset(Vision vision, Drive drive, ArrayList<Trajectory> trajectory, DifferentialDrivePoseEstimator estimator, NavSensor nav, Arm arm, Claw claw){
    
    addCommands(
        new AutoMoveArm(arm, _pos1),
        new AutoMoveClaw(claw, clawSpeed, clawRelease),
        new VisionDrive(vision, drive, trajectory.get(0), estimator, nav, 0, 0)
    );
    }
}
