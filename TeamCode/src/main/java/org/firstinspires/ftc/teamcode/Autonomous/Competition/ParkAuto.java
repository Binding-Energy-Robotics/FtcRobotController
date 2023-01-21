package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.Utilities.AutoEndPose;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Camera;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

@Autonomous(preselectTeleOp="MainTeleop")
public class ParkAuto extends LinearOpMode {
	@Override
	public void runOpMode() throws InterruptedException {
		Telemetry telemetry = new MultipleTelemetry(this.telemetry,
				FtcDashboard.getInstance().getTelemetry());

		Pose2d START = new Pose2d(0, 0, Math.toRadians(-90));
		Pose2d CENTER = new Pose2d(0, 26, Math.toRadians(-90));
		Vector2d LEFT = new Vector2d(-24, 26);
		Vector2d RIGHT = new Vector2d(24, 26);

		SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
		drive.setPoseEstimate(START);

		Claw claw = new Claw(hardwareMap);
		Flipper flipper = new Flipper(hardwareMap, telemetry);


		telemetry.addData("Status", "1");
		telemetry.update();
		Trajectory forward = drive.trajectoryBuilder(START)
				.strafeTo(CENTER.vec())
				.build();

		telemetry.addData("Status", "2");
		telemetry.update();
		Trajectory left = drive.trajectoryBuilder(CENTER)
				.strafeTo(LEFT)
				.build();

		telemetry.addData("Status", "3");
		telemetry.update();
		Trajectory right = drive.trajectoryBuilder(CENTER)
				.strafeTo(RIGHT)
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

		AutoEndPose.setPose(drive.getPoseEstimate());
	}
}
