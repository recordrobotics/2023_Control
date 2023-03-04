package org.recordrobotics.charger.commands.auto;

import org.recordrobotics.charger.commands.manual.ArmPosition;
import org.recordrobotics.charger.subsystems.Arm;
import org.recordrobotics.charger.subsystems.Drive;
import org.recordrobotics.charger.subsystems.Claw;
import org.recordrobotics.charger.subsystems.NavSensor;
import org.recordrobotics.charger.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ParallelFullAuto extends ParallelCommandGroup {
    private ArmPosition _armPosition = ArmPosition.NEUTRAL;
    


    public ParallelFullAuto(Vision vision, Drive drive, Arm arm, Claw claw, PIDController originPid, PIDController changePid, Trajectory trajectory, DifferentialDrivePoseEstimator estimator, NavSensor nav){
       
        AutoMoveArm moveArm = new AutoMoveArm(arm, originPid, changePid, _armPosition);

        addCommands(
            moveArm,

            new FullAutoSequence(vision, drive, trajectory, estimator, nav, moveArm, claw)

            
        );
    }

    
}
