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
//		FtcDashboard dashboard = FtcDashboard.getInstance();
//		Telemetry dashboardTelemetry = dashboard.getTelemetry();
//
//		TelemetryPacket packet = new TelemetryPacket();

		telemetry.addData("Status", "Initializing camera");
		telemetry.update();
//		packet.put("Status", "Initializing camera");

		Camera camera = new Camera(hardwareMap, "Webcam 1", this);

		telemetry.addData("Camera", "Initializing");
//		packet.put("Status", "Initializing camera");
		telemetry.update();

		while (!camera.isOpen() && !isStopRequested() && !opModeIsActive()) idle();

		telemetry.addData("Camera", "Open");
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		waitForStart();

		while (opModeIsActive()) {
			camera.periodic();

			telemetry.addData("Side", camera.getSide());
			telemetry.addData("Confidences", camera.getConfidences());
			telemetry.update();
		}
	}
}
