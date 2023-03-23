package org.recordrobotics.charger.commands.auto;

import org.recordrobotics.charger.commands.manual.ArmPosition;
import org.recordrobotics.charger.subsystems.Arm;
import org.recordrobotics.charger.subsystems.Claw;
import org.recordrobotics.charger.subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/*
 * This is a last resort, if nothing else about auto works
 */
public class SimpleScoreAndTaxi extends ParallelCommandGroup{
    public SimpleScoreAndTaxi(Drive drive/*, Arm arm, Claw claw, PIDController originPid, PIDController changePid, ArmPosition armPosition, double clawSpeed*/){
        addCommands(
        //new AutoMoveArm(arm, originPid, changePid, armPosition),
        //new AutoMoveClaw(claw, clawSpeed, -1),
        new AutoDrive(drive, 0.8, -3)
    );
    }

    //addisfijinished
}