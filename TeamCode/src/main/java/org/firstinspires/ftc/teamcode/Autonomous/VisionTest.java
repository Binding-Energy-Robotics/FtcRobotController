package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Framework.Vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Framework.Vision.Webcam;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class VisionTest extends LinearOpMode {
	private Webcam webcam;
	private AprilTagDetector tagDetector;
	private int[] tagIds = { 434, 435, 436 };
	private Queue<Integer> prev_detections = new LinkedList<>();

	private int identify_tag(int id) {
		for (int i = 0; i < tagIds.length; i++) {
			if (tagIds[i] == id) return i;
		}
		return -1;
	}

	@Override
	public void runOpMode() throws InterruptedException {
		tagDetector = new AprilTagDetector();

		webcam = new Webcam(hardwareMap, "Camera", tagDetector);

		while (!webcam.isOpen() && !isStarted() && !isStopRequested()) {
			idle();
		}
		while (!isStarted() && !isStopRequested()) {
			ArrayList<AprilTagDetection> detections = tagDetector.getLatestDetections();
			for (AprilTagDetection detection : detections) {
				telemetry.addData("Detection", String.valueOf(detection.id));
				int tag = identify_tag(detection.id);
				if (tag != -1) {
					prev_detections.add(tag);
				}
			}
			while (prev_detections.size() > 64) {
				prev_detections.remove();
			}
			int[] detection_counts = new int[] { 0, 0, 0 };
			for (int detection : prev_detections) {
				detection_counts[detection] += 1;
			}
			int index = 0;
			for (int i = 1; i < 3; i++) {
				if (detection_counts[index] < detection_counts[i]) {
					index = i;
				}
			}
			telemetry.addData("Side detected", index + 1);
			telemetry.update();
			idle();
		}
	}
}
