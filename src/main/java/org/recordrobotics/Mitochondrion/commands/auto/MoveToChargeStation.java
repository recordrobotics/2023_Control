package org.recordrobotics.Mitocondrion.commands.auto;

import org.recordrobotics.Mitocondrion.commands.manual.ArmPosition;
import org.recordrobotics.Mitocondrion.subsystems.Arm;
import org.recordrobotics.Mitocondrion.subsystems.Claw;
import org.recordrobotics.Mitocondrion.subsystems.Drive;
import org.recordrobotics.Mitocondrion.subsystems.NavSensor;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/*
 * This is a last resort, if nothing else about auto works
 */
public class MoveToChargeStation extends ParallelCommandGroup{
    public MoveToChargeStation(Drive drive, NavSensor nav, double nav_offset){
        addCommands(        
        new AutoDrive(drive, 0.5, -1.95),//og: -1.9, experimental values
        new SelfStationBalance(drive, nav, nav_offset)
    );
    }
}