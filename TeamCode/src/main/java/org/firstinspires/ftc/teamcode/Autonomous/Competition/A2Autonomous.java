package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.CommandGroups.DriveAndLift;
import org.firstinspires.ftc.teamcode.Framework.CommandGroups.ReleaseThenDrive;
import org.firstinspires.ftc.teamcode.Framework.Commands.AsyncDelay;
import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.Drive.Park;
import org.firstinspires.ftc.teamcode.Framework.Commands.Drive.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.Framework.Commands.Slide.SetSlidePosition;
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

	public static final Pose2d START_POSE = new Pose2d(-36, 61, Math.toRadians(-90));
	public static final Pose2d TERMINAL_TURN_A = new Pose2d(-36, 13, Math.toRadians(-90));
	public static final Pose2d TERMINAL_TURN_B = new Pose2d(-30, 6, Math.toRadians(-45));
	public static final Pose2d TERMINAL_POSE = new Pose2d(-28, 4, Math.toRadians(-45));
	public static final Pose2d TERMINAL_TURN_C = new Pose2d(-37, 12, Math.toRadians(0));
	public static final Pose2d CONE_STACK = new Pose2d(-61, 12, Math.toRadians(0));
	public static final Pose2d PARK_1 = new Pose2d(-60, 12, Math.toRadians(0));
	public static final Pose2d PARK_2 = new Pose2d(-36, 12, Math.toRadians(0));
	public static final Pose2d PARK_3 = new Pose2d(-12, 12, Math.toRadians(0));

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
				.strafeTo(TERMINAL_TURN_A.vec())
				.splineToSplineHeading(TERMINAL_TURN_B, Math.toRadians(-45))
				.strafeTo(TERMINAL_POSE.vec())
				.build();
		Trajectory terminalToStack = drive.trajectoryBuilder(TERMINAL_POSE)
				.strafeTo(TERMINAL_TURN_B.vec())
				.splineToSplineHeading(TERMINAL_TURN_C, Math.toRadians(180))
				.strafeTo(CONE_STACK.vec())
				.build();
		Trajectory stackToTerminal = drive.trajectoryBuilder(CONE_STACK)
				.strafeTo(TERMINAL_TURN_B.vec())
				.splineToSplineHeading(TERMINAL_TURN_C, Math.toRadians(180))
				.strafeTo(CONE_STACK.vec())
				.build();
		Trajectory preparePark = drive.trajectoryBuilder(TERMINAL_POSE)
				.strafeTo(TERMINAL_TURN_B.vec())
				.splineToSplineHeading(TERMINAL_TURN_C, Math.toRadians(180))
				.build();
		Trajectory park1 = drive.trajectoryBuilder(TERMINAL_TURN_C)
				.strafeTo(PARK_1.vec())
				.build();
		Trajectory park2 = drive.trajectoryBuilder(TERMINAL_TURN_C)
				.strafeTo(PARK_2.vec())
				.build();
		Trajectory park3 = drive.trajectoryBuilder(TERMINAL_TURN_C)
				.strafeTo(PARK_3.vec())
				.build();

		DriveAndLift driveStartToTerminal =
				new DriveAndLift(drive, slide, startToTerminal, LinearSlide.HIGH);
		ParallelCommandGroup driveToStack = new ParallelCommandGroup(
				new ReleaseThenDrive(drive, claw, terminalToStack),
				new SetSlidePosition(slide, LinearSlide.BOTTOM)
		);
		SequentialCommandGroup driveToTerminal = new SequentialCommandGroup(
				new CloseClaw(claw),
				new ParallelCommandGroup(
						new SetSlidePosition(slide, LinearSlide.HIGH),
						new SequentialCommandGroup(
								new AsyncDelay(0.5),
								new TrajectoryCommand(drive, stackToTerminal)
						)
				)
		);
		SequentialCommandGroup cycle = new SequentialCommandGroup(
				driveToStack,
				driveToTerminal
		);
		ParallelCommandGroup prepareToPark = new ParallelCommandGroup(
				new ReleaseThenDrive(drive, claw, preparePark),
				new SetSlidePosition(slide, LinearSlide.BOTTOM)
		);
		Park park = new Park(drive, () -> parkPosition, park1, park2, park3);

		SequentialCommandGroup autonomousRoutine = new SequentialCommandGroup(
				driveStartToTerminal,
				cycle,
				cycle,
				cycle,
				prepareToPark,
				park
		);

		schedule(autonomousRoutine);

		telemetry.addData("Status", "Initialized");
		telemetry.update();

		while (!isStarted()) {
			camera.periodic();
		}

		parkPosition = camera.getSide();
	}
}
