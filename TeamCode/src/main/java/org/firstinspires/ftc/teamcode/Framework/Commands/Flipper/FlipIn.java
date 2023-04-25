package org.firstinspires.ftc.teamcode.Framework.Commands.Flipper;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Flipper;

public class FlipIn extends InstantCommand {
	Flipper flipper;

	public FlipIn(Flipper flipper) {
		this.flipper = flipper;
	}

	@Override
	public void execute() {
		flipper.setPosition(0.98);
	}
}
