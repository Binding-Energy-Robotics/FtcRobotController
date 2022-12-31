package org.firstinspires.ftc.teamcode.Framework.Commands.Slide;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

public class SetSlidePosition extends CommandBase {
	LinearSlide slide;
	int position;

	public SetSlidePosition(LinearSlide slide, int position) {
		this.slide = slide;
		this.position = position;
	}

	@Override
	public void initialize() {
		slide.setSlidePosition(position);
	}

	@Override
	public boolean isFinished() {
		return slide.isMovementFinished();
	}
}
