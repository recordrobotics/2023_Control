package org.recordrobotics.Mitochondrion.commands.dash;

import org.recordrobotics.Mitochondrion.util.Procedure;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DashRunFunc extends CommandBase {

	private Procedure _func;

	public DashRunFunc(Procedure func) {
		_func = func;
	}

	@Override
	public void initialize() {
		_func.execute();
	}

	@Override
	public boolean isFinished() {
		return true;
	}

}
