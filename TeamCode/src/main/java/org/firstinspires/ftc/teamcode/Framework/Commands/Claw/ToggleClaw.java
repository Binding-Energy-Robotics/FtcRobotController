package org.firstinspires.ftc.teamcode.Framework.Commands.Claw;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;

public class ToggleClaw extends InstantCommand {
	private Claw claw;

	public ToggleClaw(Claw claw) {
		this.claw = claw;
		addRequirements(claw);
	}

	@Override
	public void execute(){
		this.claw.toggle();
	}
}
