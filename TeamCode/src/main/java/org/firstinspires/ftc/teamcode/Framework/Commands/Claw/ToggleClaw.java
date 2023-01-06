package org.firstinspires.ftc.teamcode.Framework.Commands.Claw;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autonomous.Tele;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;

public class ToggleClaw extends CommandBase {
	private Claw claw;

	public ToggleClaw(Claw claw) {
		this.claw = claw;
		addRequirements(claw);
	}

	@Override
	public void execute(){
		this.claw.toggle();
	}

	@Override
	public boolean isFinished(){
		return true;
	}
}
