package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Camera;

@Autonomous(name="VisionSubsystemTest", group="Test")
public class VisionSubsystemTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		FtcDashboard dash = FtcDashboard.getInstance();

		Camera camera = new Camera(hardwareMap, "Webcam 1");

		TelemetryPacket packet = new TelemetryPacket();
		packet.put("Status", "Camera Initializing");
		dash.sendTelemetryPacket(packet);

		while (!camera.isOpen() && !isStopRequested() && !opModeIsActive()) idle();

		packet = new TelemetryPacket();
		packet.put("Status", "Fully Initialized");
		dash.sendTelemetryPacket(packet);

		long time = System.nanoTime();

		while (!isStopRequested() && !opModeIsActive()) {
			camera.periodic();

			long t = System.nanoTime();
			if (t - time > 1e9) {
				time = t;
				packet = new TelemetryPacket();
				packet.put("Status", "Running");
				packet.put("Side", "Side: " + camera.getSide());
				packet.put("Confidences", "Confidences: " + camera.getConfidences());
				dash.sendTelemetryPacket(packet);
			}
		}
	}
}
