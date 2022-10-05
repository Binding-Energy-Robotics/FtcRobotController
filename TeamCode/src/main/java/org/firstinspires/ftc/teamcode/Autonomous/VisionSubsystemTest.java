package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Camera;

public class VisionSubsystemTest extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		Camera camera = new Camera(hardwareMap, "Camera");

		telemetry.addData("Camera", "Initializing");
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
