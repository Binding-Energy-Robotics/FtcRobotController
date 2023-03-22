package org.firstinspires.ftc.teamcode.Framework.Commands.Claw;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;

public class SignalBeamBreak extends CommandBase {

	private Claw claw;
	private Gamepad gamepad;

	public SignalBeamBreak(Claw claw, Gamepad gamepad) {
		this.claw = claw;
		this.gamepad = gamepad;
	}

	@Override
	public void execute() {
		if (claw.isBeamBroken() && !gamepad.isRumbling())
			gamepad.rumble(1, 1, 67);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
