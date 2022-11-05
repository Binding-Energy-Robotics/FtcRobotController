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

		TelemetryPacket packet = new TelemetryPacket();
		packet.put("Status", "Camera Initializing");
		FtcDashboard.getInstance().sendTelemetryPacket(packet);

		while (!camera.isOpen() && !isStopRequested() && !opModeIsActive()) idle();
		sleep(5000);

		packet = new TelemetryPacket();
		packet.put("Status", "Fully Initialized");
		FtcDashboard.getInstance().sendTelemetryPacket(packet);

		sleep(5000);

		waitForStart();

		while (opModeIsActive()) {
			camera.periodic();

			packet = new TelemetryPacket();
			packet.put("Status", "Running");
			packet.put("Side", "Side: " + camera.getSide());
			packet.put("Confidences", "Confidences: " + camera.getConfidences());
			FtcDashboard.getInstance().sendTelemetryPacket(packet);
		}
	}
}
