package org.recordrobotics.Mitochondrion.commands.auto;

import org.recordrobotics.Mitochondrion.commands.manual.ArmPosition;
import org.recordrobotics.Mitochondrion.subsystems.Arm;
import org.recordrobotics.Mitochondrion.subsystems.Claw;
import org.recordrobotics.Mitochondrion.subsystems.Drive;
import org.recordrobotics.Mitochondrion.subsystems.NavSensor;
import org.recordrobotics.Mitochondrion.commands.auto.ChargeStationBalance;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/*
 * This is a last resort, if nothing else about auto works
 */
public class MoveToChargeStation extends SequentialCommandGroup{
    public MoveToChargeStation(Drive drive, NavSensor nav){
        addCommands(        
        new AutoDrive(drive, 0.5, -1.5),//og: -1.9, experimental values
        new ChargeStationBalance(drive, nav)
    );
    }
}