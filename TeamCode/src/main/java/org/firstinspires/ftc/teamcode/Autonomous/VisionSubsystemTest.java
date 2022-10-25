package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Camera;

@Autonomous(name="VisionSubsystemTest", group="Test")
public class VisionSubsystemTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		Camera camera = new Camera(hardwareMap, "Webcam 1");

		FtcDashboard.getInstance().getTelemetry().addData("Camera", "Initializing");
		FtcDashboard.getInstance().getTelemetry().update();

		while (!camera.isOpen() && !isStopRequested() && !opModeIsActive()) idle();

		FtcDashboard.getInstance().getTelemetry().addData("Status", "Initialized");
		FtcDashboard.getInstance().getTelemetry().update();

		waitForStart();

		while (opModeIsActive()) {
			camera.periodic();

			FtcDashboard.getInstance().getTelemetry().addData("Side", camera.getSide());
			FtcDashboard.getInstance().getTelemetry().addData("Confidences", camera.getConfidences());
			FtcDashboard.getInstance().getTelemetry().update();
		}
	}
}
