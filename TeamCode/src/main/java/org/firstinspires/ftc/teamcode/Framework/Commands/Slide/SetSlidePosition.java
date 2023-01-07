package org.firstinspires.ftc.teamcode.Framework.Commands.Slide;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

import java.util.function.IntSupplier;

public class SetSlidePosition extends CommandBase {
	LinearSlide slide;
	IntSupplier position;

	public SetSlidePosition(LinearSlide slide, IntSupplier position) {
		this.slide = slide;
		this.position = position;
	}

	public SetSlidePosition(LinearSlide slide, int position) {
		this(slide, () -> position);
	}

	@Override
	public void initialize() {
		slide.setSlidePosition(position.getAsInt());
	}

	@Override
	public boolean isFinished() {
		return slide.isMovementFinished();
	}
}
