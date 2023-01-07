package org.firstinspires.ftc.teamcode.Framework.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

public class AsyncDelay extends CommandBase {
	private long endTime;
	private long delayNs;

	public AsyncDelay(double delaySeconds) {
		delayNs = (long)(delaySeconds * 1e9);
	}

	@Override
	public void initialize() {
		endTime = System.nanoTime() + delayNs;
	}

	@Override
	public boolean isFinished() {
		return System.nanoTime() > endTime;
	}
}
