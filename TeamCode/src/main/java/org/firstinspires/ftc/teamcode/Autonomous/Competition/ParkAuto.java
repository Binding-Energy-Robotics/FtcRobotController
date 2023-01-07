package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Camera;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

public class ParkAuto extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		Trajectory forward = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
				.forward(25)
				.build();
		Trajectory left = drive.trajectoryBuilder(new Pose2d(25, 0, 0))
				.strafeLeft(24)
				.build();
		Trajectory right = drive.trajectoryBuilder(new Pose2d(25, 0, 0))
				.strafeLeft(24)
				.build();
		Camera camera = new Camera(hardwareMap);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		while (!isStarted()) {
			camera.periodic();
			idle();
		}
		camera.stop();

		drive.followTrajectory(forward);
		int signalSide = camera.getSide();
		if (signalSide == 1) {
			drive.followTrajectory(left);
		}
		else if (signalSide == 3) {
			drive.followTrajectory(right);
		}
	}
}
