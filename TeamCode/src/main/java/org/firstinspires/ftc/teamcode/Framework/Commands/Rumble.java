package org.firstinspires.ftc.teamcode.Framework.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Rumble extends CommandBase {
	Gamepad gamepad;
	int millis;

	public Rumble(Gamepad gamepad, int millis) {
		this.gamepad = gamepad;
		this.millis = millis;
	}

	@Override
	public void execute() {
		gamepad.rumble(millis);
	}

	@Override
	public boolean isFinished() {
		return true;
	}
}
