package org.firstinspires.ftc.teamcode.Framework.Commands;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Framework.CommandGroups.DropCone;
import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.ToggleClaw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

public class DriverToggle extends InstantCommand {
	ToggleClaw toggleClaw;
	DropCone dropCone;
	LinearSlide slide;

	public DriverToggle(LinearSlide slide, Flipper flipper, Claw claw) {
		toggleClaw = new ToggleClaw(claw);
		dropCone = new DropCone(slide, flipper, claw);
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
