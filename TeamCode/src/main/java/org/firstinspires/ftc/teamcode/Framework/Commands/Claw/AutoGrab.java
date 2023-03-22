package org.firstinspires.ftc.teamcode.Framework.Commands.Claw;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

public class AutoGrab extends CommandBase {
	Claw claw;
	LinearSlide slide;
	Gamepad gamepad;

	public AutoGrab(Claw claw, LinearSlide slide, Gamepad gamepad) {
		this.claw = claw;
		this.slide = slide;
		this.gamepad = gamepad;
		addRequirements(claw);
	}

	@Override
	public void execute() {
		if (slide.isDown() && claw.grabIfConeDetected()) // short circuiting
			gamepad.rumble(1, 1, 100);
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
