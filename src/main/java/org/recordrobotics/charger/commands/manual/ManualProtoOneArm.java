// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.recordrobotics.charger.commands.manual;

import org.recordrobotics.charger.control.IControlInput;
import org.recordrobotics.charger.subsystems.ProtoOneArm;
import edu.wpi.first.wpilibj2.command.CommandBase;
 
public class ManualProtoOneArm extends CommandBase {
private ProtoOneArm _protoOneArm;
	private IControlInput _controls;
/** Creates a new ManualProtoOneArm. */
public ManualProtoOneArm(ProtoOneArm protoOneArm, IControlInput controls) {
	// Use addRequirements() here to declare subsystem dependencies.
	if (protoOneArm == null) {
			throw new IllegalArgumentException("Drive is null");
		}
		if (controls == null) {
			throw new IllegalArgumentException("Controls is null");
		}

		_protoOneArm = protoOneArm;
		_controls = controls;
		addRequirements(protoOneArm);
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
	_protoOneArm.turn(_controls.getOneArm());
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
	_protoOneArm.turn(0);
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
	return false;
}
}
