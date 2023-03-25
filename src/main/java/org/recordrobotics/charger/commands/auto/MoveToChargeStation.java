package org.recordrobotics.charger.commands.auto;

import org.recordrobotics.charger.commands.manual.ArmPosition;
import org.recordrobotics.charger.subsystems.Arm;
import org.recordrobotics.charger.subsystems.Claw;
import org.recordrobotics.charger.subsystems.Drive;
import org.recordrobotics.charger.subsystems.NavSensor;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/*
 * This is a last resort, if nothing else about auto works
 */
public class MoveToChargeStation extends ParallelCommandGroup{
    public MoveToChargeStation(Drive drive, NavSensor nav, double nav_offset){
        addCommands(        
        new AutoDrive(drive, 0.5, -1.5),//experimental values
        new SelfStationBalance(drive, nav, nav_offset)
    );
    }
}