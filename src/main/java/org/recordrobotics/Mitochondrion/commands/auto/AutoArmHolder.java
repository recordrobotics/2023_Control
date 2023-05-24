package org.recordrobotics.Mitochondrion.commands.auto;

import org.recordrobotics.Mitochondrion.commands.manual.ArmPosition;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoArmHolder  extends CommandBase{
    private AutoMoveArm _mover;
    private ArmPosition _position;
    
    public AutoArmHolder(AutoMoveArm mover, ArmPosition position){
        _mover = mover;
        _position = position;

    }

    @Override
    public void initialize() {
        _mover.setArmPosition(_position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    

}
