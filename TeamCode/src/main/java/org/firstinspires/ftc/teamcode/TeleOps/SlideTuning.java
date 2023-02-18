package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Wrist;

@Disabled
@TeleOp
public class SlideTuning extends CommandOpMode {
	LinearSlide slide;
	Wrist wrist;
	Telemetry telemetry;

	@Override
	public void initialize() {
		telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry());

		wrist = new Wrist(hardwareMap, "wrist", telemetry);
		slide = new LinearSlide(hardwareMap, telemetry, true);

		MoveWrist moveWrist = new MoveWrist(wrist, () -> 0);

		schedule(moveWrist);

		register(wrist, slide);

		telemetry.addData("Status", "Initialized");
		telemetry.update();
	}
}
