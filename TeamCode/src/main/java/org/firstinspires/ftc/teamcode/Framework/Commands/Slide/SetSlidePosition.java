package org.firstinspires.ftc.teamcode.Framework.Commands.Slide;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

import java.util.function.IntSupplier;

public class SetSlidePosition extends CommandBase {
	public static final double defaultTimeOut = 2.0;

	LinearSlide slide;
	IntSupplier position;
	double timeOut;
	ElapsedTime timer;

	public SetSlidePosition(LinearSlide slide, IntSupplier position, double timeOut) {
		this.slide = slide;
		this.position = position;
		this.timeOut = timeOut;
	}

	public SetSlidePosition(LinearSlide slide, int position, double timeOut) {
		this(slide, () -> position, timeOut);
	}

	public SetSlidePosition(LinearSlide slide, IntSupplier position) {
		this(slide, position, defaultTimeOut);
	}

	public SetSlidePosition(LinearSlide slide, int position) {
		this(slide, () -> position, defaultTimeOut);
	}

	@Override
	public void initialize() {
		slide.setSlidePosition(position.getAsInt());
		timer = new ElapsedTime();
	}

	@Override
	public boolean isFinished() {
		if (timer.time() > timeOut)
			slide.timeOut();

		return slide.isMovementFinished();
	}
}
