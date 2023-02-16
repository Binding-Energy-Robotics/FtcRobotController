package org.firstinspires.ftc.teamcode.Framework.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryUpdate extends CommandBase {
	Telemetry telemetry;
	ElapsedTime time;
	double loop;

	public TelemetryUpdate(Telemetry telemetry) {
		this.telemetry = telemetry;
		loop = 0;
	}

	@Override
	public void initialize() {
		time = new ElapsedTime();
	}

	@Override
	public void execute() {
		loop *= 0.9;
		loop += time.seconds() * 0.1;
		time.reset();
		telemetry.addData("loop time", Math.round(loop * 10000) / 10);
		telemetry.update();
	}
}
