package org.firstinspires.ftc.teamcode.Framework.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AsyncDelay extends CommandBase {
	private ElapsedTime time;
	private double delay;

	public AsyncDelay(double delaySeconds) {
		delay = delaySeconds;
	}

	@Override
	public void initialize() {
		time = new ElapsedTime();
	}

	@Override
	public boolean isFinished() {
		return time.time() > delay;
	}
}
