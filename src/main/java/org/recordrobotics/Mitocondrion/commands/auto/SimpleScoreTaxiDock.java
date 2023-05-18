package org.recordrobotics.Mitocondrion.commands.auto;

import org.recordrobotics.Mitocondrion.commands.manual.ArmPosition;
import org.recordrobotics.Mitocondrion.subsystems.Arm;
import org.recordrobotics.Mitocondrion.subsystems.Claw;
import org.recordrobotics.Mitocondrion.subsystems.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/*
 * This is a last resort, if nothing else about auto works
 */
public class SimpleScoreTaxiDock extends ParallelCommandGroup{

    public SimpleScoreTaxiDock(Drive drive/*, Arm arm, Claw claw, PIDController originPid, PIDController changePid, ArmPosition armPosition, double clawSpeed*/){
        addCommands(
        //new AutoMoveArm(arm, originPid, changePid, armPosition),
       //new AutoMoveClaw(claw, clawSpeed, -1),
        new AutoDrive(drive, 0.8, -1*Units.inchesToMeters(114.75)),
        new AutoDrive(drive, 0.8, Units.inchesToMeters(36.0))
        //return new ChargeStationBalance
    );
    }
}