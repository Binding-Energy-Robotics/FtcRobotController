package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Camera;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Wrist;

/**
 * Plan:
 * 1
 * Read apriltag and get the orientation of the cone, store for later use
 * 2
 * Drive to high junction and raise the slide.
 * 3
 * Drop slide and release cone simultaneously, once cone is released move to substation
 * 4
 * Pick up cone from substation
 * 5
 * Repeat steps 2-4 until time is almost up (skip 4 on the last iteration)
 * 6
 * Move to the correct position to park and stay there
 */
@Autonomous(preselectTeleOp="MainTeleop")
public class A2Autonomous extends CommandOpMode {
	AutoDrive drive;
	LinearSlide slide;
	Wrist wrist;
	Claw claw;
	Camera camera;

	int parkPosition = 0;

	public static final Pose2d START_POSE = new Pose2d(-63, -36, Math.toRadians(0));
	public static final Pose2d TERMINAL_TURN_POSE_A = new Pose2d(-12, -36, Math.toRadians(0));
	public static final Pose2d TERMINAL_POSE = new Pose2d(-7, -31, Math.toRadians(45));
	public static final Pose2d TERMINAL_TURN_POSE_B = new Pose2d(-12, -36, Math.toRadians(90));
	public static final Pose2d SUBSTATION_TURN_POSE = new Pose2d(-59, -36, Math.toRadians(90));
	public static final Pose2d SUBSTATION_POSE = new Pose2d(-61, -9, Math.toRadians(150));
	public static final Pose2d PARK_TRANSITION_POSE_A = new Pose2d(-12, -35, Math.toRadians(90));
	public static final Pose2d PARK_TRANSITION_POSE_B = new Pose2d(-12, -37, Math.toRadians(90));
	public static final Pose2d PARK_POSE_1 = new Pose2d(-12, -12, Math.toRadians(90));
	public static final Pose2d PARK_POSE_2 = new Pose2d(-61, -9, Math.toRadians(45));
	public static final Pose2d PARK_POSE_3 = new Pose2d(-61, -9, Math.toRadians(90));

	@Override
	public void initialize() {
		Telemetry telemetry = new MultipleTelemetry(this.telemetry);

		drive = new AutoDrive(hardwareMap);
		slide = new LinearSlide(hardwareMap, telemetry, true);
		wrist = new Wrist(hardwareMap, telemetry);
		claw = new Claw(hardwareMap);
		camera = new Camera(hardwareMap);

		register(drive, slide, wrist, claw, camera);

		// generate trajectories
		Trajectory startToTerminal = drive.trajectoryBuilder(START_POSE)
				.lineTo(TERMINAL_TURN_POSE_A.vec())
				.splineToSplineHeading(TERMINAL_POSE, TERMINAL_POSE.getHeading())
				.build();
		Trajectory terminalToCones = drive.trajectoryBuilder(TERMINAL_POSE)
				.splineToSplineHeading(TERMINAL_TURN_POSE_B, Math.toRadians(180))
				.lineTo(SUBSTATION_TURN_POSE.vec())
				.splineToSplineHeading(SUBSTATION_POSE, SUBSTATION_POSE.getHeading())
				.build();
		Trajectory conesToTerminal = drive.trajectoryBuilder(SUBSTATION_POSE)
				.splineToSplineHeading(SUBSTATION_TURN_POSE, Math.toRadians(0))
				.lineTo(TERMINAL_TURN_POSE_A.vec())
				.splineToSplineHeading(TERMINAL_POSE, TERMINAL_POSE.getHeading())
				.build();
		Trajectory park1 = drive.trajectoryBuilder(TERMINAL_POSE)
				.splineToSplineHeading(PARK_TRANSITION_POSE_A, Math.toRadians(135))
				.splineToSplineHeading(PARK_POSE_1, Math.toRadians(90))
				.build();
		Trajectory park2 = drive.trajectoryBuilder(TERMINAL_POSE)
				.splineToSplineHeading(PARK_POSE_2, PARK_POSE_2.getHeading())
				.build();
		Trajectory park3 = drive.trajectoryBuilder(TERMINAL_POSE)
				.splineToSplineHeading(PARK_TRANSITION_POSE_B, Math.toRadians(-120))
				.splineToSplineHeading(PARK_POSE_3, Math.toRadians(90))
				.build();

		while (!isStarted()) {
			camera.periodic();
		}

		parkPosition = camera.getSide();
	}
}
