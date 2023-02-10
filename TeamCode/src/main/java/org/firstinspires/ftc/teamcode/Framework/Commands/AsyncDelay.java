package org.firstinspires.ftc.teamcode.Framework.Commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AsyncDelay extends CommandBase {
	public interface ElapsedTimeSupplier {
		ElapsedTime get();
	}

	private ElapsedTimeSupplier supplier = null;
	private ElapsedTime time;
	private double delay;

	public AsyncDelay(double delaySeconds) {
		delay = delaySeconds;
	}

	public AsyncDelay(ElapsedTimeSupplier timer, double stopTime) {
		supplier = timer;
		delay = stopTime;
	}

	@Override
	public void initialize() {
		if (supplier == null) {
			time = new ElapsedTime();
		}
		else {
			time = supplier.get();
		}
	}

	@Override
	public boolean isFinished() {
		return time.time() > delay;
	}
}
