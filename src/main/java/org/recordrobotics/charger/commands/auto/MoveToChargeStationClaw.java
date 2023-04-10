package org.recordrobotics.charger.commands.auto;

import org.recordrobotics.charger.commands.manual.ArmPosition;
import org.recordrobotics.charger.subsystems.Arm;
import org.recordrobotics.charger.subsystems.Claw;
import org.recordrobotics.charger.subsystems.Drive;
import org.recordrobotics.charger.subsystems.NavSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/*
 * This is a last resort, if nothing else about auto works
 */
public class MoveToChargeStationClaw extends ParallelCommandGroup{
    public MoveToChargeStationClaw(Drive drive, Arm arm, Claw claw, ArmPosition armPosition, NavSensor nav, double nav_offset){
        addCommands(
        //new AutoMoveArm(arm, armPosition),
        //new AutoMoveClaw(claw, 0.3, -1),
        new AutoDrive(drive, 0.2, -1.1),
        new SelfStationBalance(drive, nav, nav_offset)
    );
    }
}