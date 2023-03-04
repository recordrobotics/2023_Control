package org.recordrobotics.charger.commands.auto;

import org.recordrobotics.charger.subsystems.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoSequenceOuter extends SequentialCommandGroup{

    public AutoSequenceOuter(Vision vision, Drive drive, Arm arm, Claw claw, PIDController originPid, PIDController changePid, Trajectory trajectory, DifferentialDrivePoseEstimator estimator, NavSensor nav){
        
        addCommands(
            new AutoMoveClaw(claw, 0.2, 1),

            new ParallelFullAuto(vision, drive, arm, claw, originPid, changePid, trajectory, estimator, nav)
        );

    }

}