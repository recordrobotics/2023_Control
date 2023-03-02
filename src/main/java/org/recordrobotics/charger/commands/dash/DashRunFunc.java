package org.recordrobotics.charger.commands.dash;

import org.recordrobotics.charger.util.Procedure;

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
