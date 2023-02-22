package org.recordrobotics.charger.commands.group;

import org.recordrobotics.charger.subsystems.Claw;
import org.recordrobotics.charger.commands.auto.AutoMoveClaw;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class paralGrabCube extends ParallelCommandGroup{
    private static final double speed = 0.2;

    public paralGrabCube(Claw claw /*, Arm arm */){
        addCommands(
            new AutoMoveClaw(claw, speed)
        );
    }
    
}
