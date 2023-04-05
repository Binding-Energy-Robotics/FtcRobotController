package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.Utilities.LogData;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

@Disabled
@TeleOp
public class SlideDataCollection extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		Telemetry telemetry =
				new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), super.telemetry);
		LinearSlide slide = new LinearSlide(hardwareMap, telemetry, false);
		Flipper flipper = new Flipper(hardwareMap, telemetry);
		flipper.setPosition(0);
		LogData.open();

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		while (opModeIsActive()) {
			double power = (gamepad1.right_trigger - gamepad1.left_trigger) / 3;
			LogData.addData("power", power);
			LogData.addData("position", slide.getEncoder());
			LogData.update();
			slide.setPower(power);
		}

		LogData.close();
	}
}
