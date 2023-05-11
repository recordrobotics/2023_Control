package org.recordrobotics.Mitocondrion.commands.auto;

import org.recordrobotics.Mitocondrion.commands.manual.ArmPosition;
import org.recordrobotics.Mitocondrion.subsystems.Arm;
import org.recordrobotics.Mitocondrion.subsystems.Claw;
import org.recordrobotics.Mitocondrion.subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/*
 * This is a last resort, if nothing else about auto works
 */
public class SimpleScoreAndTaxi extends ParallelCommandGroup{
    public SimpleScoreAndTaxi(Drive drive, Arm arm, Claw claw, ArmPosition armPosition){
        addCommands(
        //new AutoMoveArm(arm, armPosition),
        //new AutoMoveClaw(claw, 0.3, -1),
        new AutoDrive(drive, 0.2, -3)
    );
    }
}