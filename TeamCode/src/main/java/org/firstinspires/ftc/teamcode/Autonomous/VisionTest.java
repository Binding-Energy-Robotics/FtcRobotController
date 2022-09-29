package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Framework.Vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Framework.Vision.Camera;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class VisionTest extends LinearOpMode {
	private Camera webcam;
	private AprilTagDetector tagDetector;

	@Override
	public void runOpMode() throws InterruptedException {
		tagDetector = new AprilTagDetector(2.75 / 100.0, true);

		webcam = new Camera(hardwareMap, "Camera", tagDetector, this);

		while (!webcam.isOpen() && !isStarted() && !isStopRequested()) {
			idle();
		}
		while (!isStarted() && !isStopRequested()) {
			ArrayList<AprilTagDetection> detections = tagDetector.getLatestDetections();
			for (AprilTagDetection detection : detections) {
				telemetry.addData("Detection", String.valueOf(detection.id));
			}
			telemetry.update();
			idle();
		}
	}
}
