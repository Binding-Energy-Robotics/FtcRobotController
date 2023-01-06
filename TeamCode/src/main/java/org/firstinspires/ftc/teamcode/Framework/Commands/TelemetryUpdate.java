package org.firstinspires.ftc.teamcode.Framework.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryUpdate extends CommandBase {
	Telemetry telemetry;

	public TelemetryUpdate(Telemetry telemetry) {
		this.telemetry = telemetry;
	}

	@Override
	public void execute() {
		telemetry.update();
	}
}
