package org.recordrobotics.Mitochondrion.commands.auto;

import org.recordrobotics.Mitochondrion.commands.manual.ArmPosition;
import org.recordrobotics.Mitochondrion.subsystems.Arm;
import org.recordrobotics.Mitochondrion.subsystems.Claw;
import org.recordrobotics.Mitochondrion.subsystems.Drive;
import org.recordrobotics.Mitochondrion.subsystems.NavSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/*
 * This is a last resort, if nothing else about auto works
 */
public class MoveToChargeStationClaw extends ParallelCommandGroup{
    public MoveToChargeStationClaw(Drive drive, Arm arm, /*Claw claw,*/ ArmPosition armPosition, NavSensor nav, double nav_offset){
        addCommands(
        //new AutoMoveArm(arm, armPosition),
        //new AutoMoveClaw(claw, 0.3, -1),
        new AutoDrive(drive, 0.2, -1.1),
        new SelfStationBalance(drive, nav, nav_offset)
    );
    }
}