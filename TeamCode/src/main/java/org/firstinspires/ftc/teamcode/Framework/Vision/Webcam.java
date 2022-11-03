package org.firstinspires.ftc.teamcode.Framework.Vision;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Iterator;

/**
 * This class will initialize a camera from the robot and
 * binds an OpenCvPipeline instance, which actually does the image processing.
 * To use this class, first create a pipeline class in the subsystems directory.
 * This pipeline will be what processes all of the camera data and will be
 * what the main program queries for information from the camera.
 * Initialize the pipeline first and pass it as an argument to the camera during initialization.
 * Call stop on the camera immediately after you have used the camera for the last time.
 * The camera will take several seconds to close, but it will close asynchronously
 * so it will not slow down the main code execution.
 * If your op mode ends while the camera is active or while it is still closing,
 * the app will force a crash to kill the program and will then restart.
 * Restarting the app can take upwards of 30 seconds which
 * can severely decrease the number of points scored in tele-op.
 * In some cases, the app can also fail to reconnect to the robot once it restarts,
 * preventing you from scoring any points in tele-op or endgame.
 * Moral of the story: close the camera once you're done using it or bad things happen.
 *
 * @author Alex Prichard
 */
public class Webcam {
	private OpenCvCamera camera;

	private volatile boolean open = false;
	private volatile boolean viewportRunning = false;
	public volatile boolean isStopped = false;

	public Webcam(HardwareMap hw, String name, OpenCvPipeline pipeline) {
		int cameraMonitorViewId = hw.appContext.getResources().getIdentifier(
				"cameraMonitorViewId", "id", hw.appContext.getPackageName());
		FtcDashboard.getInstance().getTelemetry().addData("View", cameraMonitorViewId);
		FtcDashboard.getInstance().getTelemetry().addData("Name", name);
		FtcDashboard.getInstance().getTelemetry().addData("Size", hw.size());
		for (int i = 0; i < hw.allDeviceMappings.size(); i++) {
			for (Iterator<? extends HardwareDevice> iter = hw.allDeviceMappings.get(i).iterator(); iter.hasNext(); ) {
				HardwareDevice device = iter.next();
				FtcDashboard.getInstance().getTelemetry().addData(String.valueOf(i), device.getDeviceName());
			}
		}
		FtcDashboard.getInstance().getTelemetry().update();

		WebcamName webcamName = hw.get(WebcamName.class, name);
		FtcDashboard.getInstance().getTelemetry().addData("Test", "1");
		FtcDashboard.getInstance().getTelemetry().addData("View", cameraMonitorViewId);
		FtcDashboard.getInstance().getTelemetry().addData("Webcam attached", webcamName.isAttached());
		FtcDashboard.getInstance().getTelemetry().addData("Webcam name", webcamName.getUsbDeviceNameIfAttached());
		FtcDashboard.getInstance().getTelemetry().update();

		camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
			@Override
			public void onOpened() {
				camera.startStreaming(1280, 720);
				camera.setViewportRenderer(OpenCvCamera.ViewportRenderer.GPU_ACCELERATED);
				camera.setPipeline(pipeline);
				open = true;
			}

			@Override
			public void onError(int errorCode) {
				Log.e("Webcam failed to open", String.valueOf(errorCode));
			}
		});
	}

	public boolean isOpen() { return open; }

	public boolean isViewportRunning() { return viewportRunning; }

	public void pauseViewport() {
		if (viewportRunning) {
			viewportRunning = false;
			camera.pauseViewport();
		}
	}

	public void resumeViewport() {
		if (!viewportRunning) {
			viewportRunning = true;
			camera.resumeViewport();
		}
	}

	public void stop() {
		if (!isStopped) {
			camera.closeCameraDeviceAsync(()->{});
			open = false;
			isStopped = true;
		}
	}
}