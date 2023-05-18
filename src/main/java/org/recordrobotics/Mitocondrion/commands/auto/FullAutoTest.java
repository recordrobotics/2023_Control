package org.recordrobotics.Mitocondrion.commands.auto;

import java.util.ArrayList;

import org.recordrobotics.Mitocondrion.commands.manual.ArmPosition;
import org.recordrobotics.Mitocondrion.subsystems.Drive;
import org.recordrobotics.Mitocondrion.subsystems.NavSensor;
import org.recordrobotics.Mitocondrion.subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class FullAutoTest extends ParallelCommandGroup {
    private ArmPosition _armPosition = ArmPosition.NEUTRAL;
    


    public FullAutoTest(Vision vision, Drive drive, PIDController originPid, PIDController changePid, ArrayList<Trajectory> trajectory, DifferentialDrivePoseEstimator estimator, NavSensor nav){
       
        System.out.println("auto move arm");

        addCommands(
            //moveArm,

            new FullAutoSequenceTest(vision, drive, trajectory, estimator, nav)
        );
    }

    
}