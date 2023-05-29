package org.recordrobotics.Mitochondrion.commands.auto;

import org.recordrobotics.Mitochondrion.commands.manual.ArmPosition;
import org.recordrobotics.Mitochondrion.subsystems.Arm;
import org.recordrobotics.Mitochondrion.subsystems.Claw;
import org.recordrobotics.Mitochondrion.subsystems.Drive;
import org.recordrobotics.Mitochondrion.subsystems.NavSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/*
 * This is a last resort, if nothing else about auto works
 */
public class SimpleScoreTaxiDock extends SequentialCommandGroup{

    public SimpleScoreTaxiDock(Drive drive, NavSensor nav, Arm arm, Claw claw, PIDController originPid, PIDController changePid, ArmPosition armPosition, double clawSpeed){
        addCommands(
        //new AutoMoveArm(arm, originPid, changePid, armPosition),
       //new AutoMoveClaw(claw, clawSpeed, -1),
        new AutoDrive(drive, 0.8, -1*Units.inchesToMeters(114.75)),
        new AutoDrive(drive, 0.8, Units.inchesToMeters(36.0)),
        new ChargeStationBalance(drive, nav)
    );
    }
}