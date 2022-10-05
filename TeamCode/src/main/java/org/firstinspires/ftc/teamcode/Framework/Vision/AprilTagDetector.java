package org.firstinspires.ftc.teamcode.Framework.Vision;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagDetector extends OpenCvPipeline {
	private long nativeApriltagPtr;
	private Mat grey = new Mat();
	private volatile ArrayList<AprilTagDetection> detections = new ArrayList<>();

	double fx;
	double fy;
	double cx;
	double cy;

	boolean drawBoxes;

	// UNITS ARE METERS
	double tagsize;
	double tagsizeX;
	double tagsizeY;

	private volatile boolean newData = false;

	public AprilTagDetector(double tagsize, double fx, double fy, double cx, double cy, boolean draw) {
		this.tagsize = tagsize;
		this.tagsizeX = tagsize;
		this.tagsizeY = tagsize;
		this.fx = fx;
		this.fy = fy;
		this.cx = cx;
		this.cy = cy;
		this.drawBoxes = draw;

		// Allocate a native context object. See the corresponding deletion in the finalizer
		nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(
				AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 1
		);
	}

	public AprilTagDetector(double tagsize, boolean draw) {
		this(tagsize, 1430, 1430, 640, 360, draw);
	}

	public AprilTagDetector(double tagsize) {
		this(tagsize, false);
	}

	public AprilTagDetector() {
		this(0.032, true);
	}

	@Override
	public void finalize() {
		// Might be null if createApriltagDetector() threw an exception
		if(nativeApriltagPtr != 0) {
			// Delete the native context we created in the constructor
			AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
			nativeApriltagPtr = 0;
		}
		else {
			System.out.println("AprilTagDetectionPipeline.finalize(): nativeApriltagPtr was NULL");
		}
	}

	@Override
	public Mat processFrame(Mat input) {
		// Convert to greyscale
		Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

		// Run AprilTag
		detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grey, tagsize, fx, fy, cx, cy);
		newData = true;

		if (drawBoxes) {
			for (int i = 0; i < detections.size(); i++) {
				addBox(input, detections.get(i).corners);
				addId(input, detections.get(i));
			}
		}

		return input;
	}

	public boolean isNewData() { return newData; }

	public ArrayList<AprilTagDetection> getLatestDetections() {
		newData = false;
		return detections;
	}

	private void addBox(Mat image, Point[] points) {
		for (int i = 0; i < 4; i++) {
			Imgproc.line(image, points[i], points[(i + 1) % 4], new Scalar(255, 255, 0), 3);
		}
	}

	private void addId(Mat image, AprilTagDetection detection) {
		Imgproc.putText(image, String.valueOf(detection.id),
				detection.corners[2], 0, 1, new Scalar(255, 0, 128), 2);
	}
}