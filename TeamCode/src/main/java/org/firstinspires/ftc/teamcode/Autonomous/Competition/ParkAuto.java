package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Camera;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

@Autonomous(preselectTeleOp="MainTeleop")
public class ParkAuto extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

		Claw claw = new Claw(hardwareMap);
		Flipper flipper = new Flipper(hardwareMap, telemetry);


		telemetry.addData("Status", "1");
		telemetry.update();
		Trajectory forward = drive.trajectoryBuilder(new Pose2d(0, 0, 0))
				.back(25)
				.build();

		telemetry.addData("Status", "2");
		telemetry.update();
		Trajectory left = drive.trajectoryBuilder(new Pose2d(-25, 0, 0))
				.strafeLeft(24)
				.build();

		telemetry.addData("Status", "3");
		telemetry.update();
		Trajectory right = drive.trajectoryBuilder(new Pose2d(-25, 0, 0))
				.strafeRight(24)
				.build();

		telemetry.addData("Status", "4");
		telemetry.update();
		Camera camera = new Camera(hardwareMap);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		while (!isStarted()) {
			camera.periodic();
			telemetry.addData("Confidences", camera.getConfidences());
			telemetry.update();
			idle();
		}
		camera.stop();

		int signalSide = camera.getSide();

		telemetry.addData("Signal", signalSide);
		telemetry.update();

		drive.followTrajectory(forward);
		if (signalSide == 1) {
			drive.followTrajectory(left);
		}
		else if (signalSide == 3) {
			drive.followTrajectory(right);
		}
	}
}
