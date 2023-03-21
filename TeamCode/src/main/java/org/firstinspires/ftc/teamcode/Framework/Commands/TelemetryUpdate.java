package org.firstinspires.ftc.teamcode.Framework.Commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.Utilities.LogData;

public class TelemetryUpdate extends CommandBase {
	Telemetry telemetry;

	ElapsedTime time;
	double loop;

	double previousLoop;
	double targetLoop;

	public TelemetryUpdate(Telemetry telemetry, double loopPeriod) {
		this.telemetry = telemetry;
		loop = 0;
		targetLoop = loopPeriod;
		previousLoop = 0;
	}

	public TelemetryUpdate(Telemetry telemetry) {
		this(telemetry, 0.05);
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

		while (previousLoop + targetLoop < time.seconds());
		previousLoop += targetLoop;

		LogData.update();
	}
}
