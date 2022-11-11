package org.firstinspires.ftc.teamcode.Framework.Commands.Wrist;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Wrist;

import java.util.function.DoubleSupplier;

public class MoveWrist extends CommandBase {
	private Wrist wrist;
	private DoubleSupplier position;

	public MoveWrist(Wrist wrist, DoubleSupplier position) {
		this.wrist = wrist;
		this.position = position;
		addRequirements(wrist);
	}

	@Override
	public void execute(){
		wrist.setPosition(position.getAsDouble());
	}

	@Override
	public boolean isFinished(){
		return false;
	}
}
