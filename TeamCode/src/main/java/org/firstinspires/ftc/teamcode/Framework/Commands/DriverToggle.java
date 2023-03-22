package org.firstinspires.ftc.teamcode.Framework.Commands;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Framework.CommandGroups.DropCone;
import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.AutoGrab;
import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.ToggleClaw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

public class DriverToggle extends InstantCommand {
	Command toggleClaw;
	Command dropCone;
	LinearSlide slide;

	public DriverToggle(LinearSlide slide, Flipper flipper, Claw claw, Gamepad gamepad) {
		toggleClaw = new ToggleClaw(claw).andThen(new AutoGrab(claw, slide, gamepad));
		dropCone = new DropCone(slide, flipper, claw).andThen(new AutoGrab(claw, slide, gamepad));
		this.slide = slide;
	}

	@Override
	public void initialize() {
		if (slide.isDown()) {
			CommandScheduler.getInstance().schedule(toggleClaw);
		}
		else {
			CommandScheduler.getInstance().schedule(dropCone);
		}
	}
}
