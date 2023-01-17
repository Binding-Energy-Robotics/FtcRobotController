package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Framework.Vision.AprilTagDetector;
import org.firstinspires.ftc.teamcode.Framework.Vision.Webcam;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class Camera extends SubsystemBase {
	private Webcam webcam;
	private AprilTagDetector tagDetector;

	private int[] tagIds = new int[] { 434, 435, 436 };
	private double[] confidences = new double[] { 1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0 };
	private static final double UPDATE_WEIGHT = 0.1;

	public Camera(HardwareMap hw, String webcamName) {
		tagDetector = new AprilTagDetector(true);
		webcam = new Webcam(hw, webcamName, tagDetector);
	}

	public Camera(HardwareMap hw) {
		this(hw, "Webcam 1");
	}

	public int getSide() {
		int index = 0;
		for (int i = 1; i < 3; i++) {
			if (confidences[index] < confidences[i])
				index = i;
		}
		return index + 1;
	}

	public String getConfidences() {
		double[][] confidencePairings = new double[3][2]; // bundle index and confidence for sorting
		for (int i = 0; i < 3; i++) {
			confidencePairings[i] = new double[] { i + 1, confidences[i] };
		}

		for (int i = 2; i > 0; i--) { // bubble sort
			for (int j = 0; j < i; j++) {
				if (confidencePairings[j][1] > confidencePairings[j + 1][1]) {
					double[] first = confidencePairings[j];
					confidencePairings[j] = confidencePairings[j + 1];
					confidencePairings[j + 1] = first;
				}
			}
		}

		StringBuilder output = new StringBuilder("{"); // format string
		for (int i = 0; i < 3; i++) {
			output.append("\nSide: ")
					.append((int) confidencePairings[i][0])
					.append(", Confidence: ")
					.append((int) (confidencePairings[i][1] * 1000) / 10.0)
					.append(";");
		}
		output.append("\n}");
		return output.toString();
	}

	@Override
	public void periodic() {
		if (!webcam.isOpen()) return;
		if (!tagDetector.isNewData()) return;

		ArrayList<AprilTagDetection> detections = tagDetector.getLatestDetections();

		double[] detected = new double[] { 0, 0, 0 };
		for (AprilTagDetection detection : detections) {
			for (int i = 0; i < 3; i++) {
				if (tagIds[i] == detection.id) {
					detected[i] = UPDATE_WEIGHT;
					break;
				}
			}
		}

		double acc = 0;
		for (int i = 0; i < 3; i++) {
			confidences[i] += detected[i];
			acc += confidences[i];
		}
		for (int i = 0; i < 3; i++) {
			confidences[i] /= acc;
		}
	}

	public boolean isOpen() { return webcam.isOpen(); }

	public void stop() { webcam.stop(); }
}
