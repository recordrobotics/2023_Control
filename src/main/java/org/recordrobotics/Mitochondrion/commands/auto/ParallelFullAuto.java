package org.recordrobotics.Mitochondrion.commands.auto;

import java.util.ArrayList;

import org.recordrobotics.Mitochondrion.commands.manual.ArmPosition;
import org.recordrobotics.Mitochondrion.subsystems.Arm;
import org.recordrobotics.Mitochondrion.subsystems.Drive;
import org.recordrobotics.Mitochondrion.subsystems.Claw;
import org.recordrobotics.Mitochondrion.subsystems.NavSensor;
import org.recordrobotics.Mitochondrion.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class ParallelFullAuto extends ParallelCommandGroup {
    private ArmPosition _armPosition = ArmPosition.NEUTRAL;
    


    public ParallelFullAuto(Vision vision, Drive drive, Arm arm, Claw claw, PIDController originPid, PIDController changePid, ArrayList<Trajectory> trajectory, DifferentialDrivePoseEstimator estimator, NavSensor nav){
       
      //  AutoMoveArm moveArm = new AutoMoveArm(arm);

        addCommands(
            //moveArm,

            //new FullAutoSequence(vision, drive, trajectory, estimator, nav, moveArm, claw)

            
        );
    }

    
}
