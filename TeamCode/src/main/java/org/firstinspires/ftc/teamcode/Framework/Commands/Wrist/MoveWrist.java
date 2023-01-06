package org.firstinspires.ftc.teamcode.Framework.Commands.Wrist;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Wrist;

import java.util.function.DoubleSupplier;

public class MoveWrist extends CommandBase {
	private Wrist wrist;
	private DoubleSupplier power;
	private double position;
	private double prevNanos;

	public MoveWrist(Wrist wrist, DoubleSupplier power) {
		this.wrist = wrist;
		this.power = power;
		this.position = 0.95;
		this.wrist.setPosition(this.position);
		this.prevNanos = System.nanoTime();
		addRequirements(wrist);
	}

	@Override
	public void execute(){
		double nanos = System.nanoTime();
		double dt = (nanos - prevNanos) * 1e-9;
		if (dt >= 1e-2) {
			prevNanos = nanos;
			position -= dt * power.getAsDouble();
			position = Range.clip(position, 0.65, 0.95);
			wrist.setPosition(position);
		}
	}

	@Override
	public boolean isFinished(){
		return false;
	}
}
