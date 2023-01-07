package org.firstinspires.ftc.teamcode.Framework.Commands.Flipper;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Flipper;

public class FlipOut extends CommandBase {
	Flipper flipper;

	public FlipOut(Flipper flipper) {
		this.flipper = flipper;
	}

	@Override
	public void execute() {
		flipper.setPosition(1);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
