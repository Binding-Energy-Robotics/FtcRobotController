package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.Commands.AsyncDelay;
import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.OpenClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.Drive.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.Framework.Commands.Flipper.FlipIn;
import org.firstinspires.ftc.teamcode.Framework.Commands.Flipper.FlipOut;
import org.firstinspires.ftc.teamcode.Framework.Commands.SavePosition;
import org.firstinspires.ftc.teamcode.Framework.Commands.Slide.SetSlidePosition;
import org.firstinspires.ftc.teamcode.Framework.Commands.TelemetryUpdate;
import org.firstinspires.ftc.teamcode.Framework.Utilities.AutoEndPose;
import org.firstinspires.ftc.teamcode.Framework.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Camera;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

import java.util.HashMap;

@Config
@Autonomous(preselectTeleOp = "MainTeleop")
public class LeftAuto extends CommandOpMode {
	public static Pose2d START_POSE = new Pose2d(-31, -64.5, Math.toRadians(-90));
	public static Pose2d START_POSE_A = new Pose2d(-34, -48, Math.toRadians(120));
	public static Pose2d START_POSE_B = new Pose2d(-36, -24, Math.toRadians(80));

	public static Pose2d SCORE_POSE_ZERO = new Pose2d(-25.5, -9.5, Math.toRadians(45));
	public static Pose2d SCORE_POSE_ONE = new Pose2d(-25.5, -9.5, Math.toRadians(45));
	public static Pose2d SCORE_POSE_TWO = new Pose2d(-25.5, -9.5, Math.toRadians(45));
	public static Pose2d SCORE_POSE_THREE = new Pose2d(-25.5, -9.5, Math.toRadians(45));
	public static Pose2d SCORE_POSE_FOUR = new Pose2d(-25.5, -9.5, Math.toRadians(45));
	public static Pose2d SCORE_POSE_FIVE = new Pose2d(-25.5, -9.5, Math.toRadians(45));

	public static Pose2d CONE_POSE_ONE = new Pose2d(-57.5, -14, Math.toRadians(0));
	public static Pose2d CONE_POSE_TWO = new Pose2d(-57, -14.33, Math.toRadians(0));
	public static Pose2d CONE_POSE_THREE = new Pose2d(-57, -14.67, Math.toRadians(0));
	public static Pose2d CONE_POSE_FOUR = new Pose2d(-57, -15, Math.toRadians(0));
	public static Pose2d CONE_POSE_FIVE = new Pose2d(-57, -15.33, Math.toRadians(0));

	public static Pose2d ZONE_ONE = new Pose2d(-52, -16, Math.toRadians(0));
	public static Pose2d ZONE_TWO = new Pose2d(-32, -16, Math.toRadians(90));
	public static Pose2d ZONE_THREE = new Pose2d(-8, -16, Math.toRadians(90));

	Telemetry telemetry;

	AutoDrive drive;
	LinearSlide slide;
	Flipper flipper;
	Claw claw;
	Camera camera;

	public static int signalSide = 1;

	private Command cycle(int coneHeight, Trajectory junctionToCones, Trajectory conesToJunction) {
		return new SequentialCommandGroup(
				new ParallelCommandGroup(
						new SetSlidePosition(slide, LinearSlide.CONE_STACK[coneHeight]),
						new SequentialCommandGroup(
								new AsyncDelay(0.25),
								new OpenClaw(claw),
								new AsyncDelay(0.1),
								new FlipIn(flipper)
						),
						new SequentialCommandGroup(
								new AsyncDelay(0.2),
								new TrajectoryCommand(drive, junctionToCones)
						)
				),
				new CloseClaw(claw),
				new AsyncDelay(0.25),
				new ParallelCommandGroup(
						new SetSlidePosition(slide, LinearSlide.LOW),
						new SequentialCommandGroup(
								new AsyncDelay(0.15),
								new ParallelCommandGroup(
										new TrajectoryCommand(drive, conesToJunction),
										new SequentialCommandGroup(
												new AsyncDelay(0.1),
												new ParallelCommandGroup(
														new SetSlidePosition(slide, LinearSlide.TWO_CONE),
														new SequentialCommandGroup(
																new AsyncDelay(0.5),
																new ParallelCommandGroup(
																		new SetSlidePosition(slide, LinearSlide.HIGH),
																		new SequentialCommandGroup(
																				new AsyncDelay(0.25),
																				new FlipOut(flipper)
																		)
																)
														)
												)
										)
								)
						)
				)
		);
	}

	@Override
	public void initialize() {
		telemetry = new MultipleTelemetry(super.telemetry,
				FtcDashboard.getInstance().getTelemetry());

		drive = new AutoDrive(hardwareMap, START_POSE);
		slide = new LinearSlide(hardwareMap, telemetry);
		flipper = new Flipper(hardwareMap, telemetry);
		claw = new Claw(hardwareMap);
		claw.open();
		camera = new Camera(hardwareMap);

		register(drive, slide, flipper, claw);

		Trajectory junctionToConesOne = drive.trajectoryBuilder(SCORE_POSE_ZERO, true)
				.splineTo(CONE_POSE_ONE.vec(), Math.toRadians(180))
				.build();
		Trajectory conesToJunctionOne = drive.trajectoryBuilder(
				junctionToConesOne.end(), Math.toRadians(0))
				.splineTo(SCORE_POSE_ONE.vec(), SCORE_POSE_ONE.getHeading())
				.build();

		Trajectory junctionToConesTwo = drive.trajectoryBuilder(conesToJunctionOne.end(), true)
				.splineTo(CONE_POSE_TWO.vec(), Math.toRadians(180))
				.build();
		Trajectory conesToJunctionTwo = drive.trajectoryBuilder(
				junctionToConesTwo.end(), Math.toRadians(0))
				.splineTo(SCORE_POSE_TWO.vec(), SCORE_POSE_TWO.getHeading())
				.build();

		Trajectory junctionToConesThree = drive.trajectoryBuilder(conesToJunctionTwo.end(), true)
				.splineTo(CONE_POSE_THREE.vec(), Math.toRadians(180))
				.build();
		Trajectory conesToJunctionThree = drive.trajectoryBuilder(
				junctionToConesThree.end(), Math.toRadians(0))
				.splineTo(SCORE_POSE_THREE.vec(), SCORE_POSE_THREE.getHeading())
				.build();

		Trajectory junctionToConesFour = drive.trajectoryBuilder(conesToJunctionThree.end(), true)
				.splineTo(CONE_POSE_FOUR.vec(), Math.toRadians(180))
				.build();
		Trajectory conesToJunctionFour = drive.trajectoryBuilder(
				junctionToConesFour.end(), Math.toRadians(0))
				.splineTo(SCORE_POSE_FOUR.vec(), SCORE_POSE_FOUR.getHeading())
				.build();

		Trajectory junctionToConesFive = drive.trajectoryBuilder(conesToJunctionFour.end(), true)
				.splineTo(CONE_POSE_FIVE.vec(), Math.toRadians(180))
				.build();
		Trajectory conesToJunctionFive = drive.trajectoryBuilder(junctionToConesFive.end(), Math.toRadians(0))
				.splineTo(SCORE_POSE_FIVE.vec(), SCORE_POSE_FIVE.getHeading())
				.build();


		TrajectoryCommand startToJunction = new TrajectoryCommand(drive,
				drive.trajectoryBuilder(START_POSE, Math.toRadians(90))
				.splineTo(START_POSE_A.vec(), START_POSE_A.getHeading())
				.splineToSplineHeading(START_POSE_B, Math.toRadians(75))
				.splineToSplineHeading(SCORE_POSE_ZERO, SCORE_POSE_ZERO.getHeading())
				.build()
		);
		TrajectoryCommand parkOne = new TrajectoryCommand(drive,
				drive.trajectoryBuilder(SCORE_POSE_FIVE, true)
				.splineToSplineHeading(ZONE_ONE, Math.toRadians(180))
				.build()
		);
		TrajectoryCommand parkTwo = new TrajectoryCommand(drive,
				drive.trajectoryBuilder(SCORE_POSE_FIVE)
				.lineToLinearHeading(ZONE_TWO)
				.build()
		);
		TrajectoryCommand parkThree = new TrajectoryCommand(drive,
				drive.trajectoryBuilder(SCORE_POSE_FIVE, Math.toRadians(-45))
				.splineToSplineHeading(ZONE_THREE, Math.toRadians(0))
				.build()
		);


		ParallelCommandGroup setUpScoring = new ParallelCommandGroup(
				startToJunction,
				new CloseClaw(claw),
				new SequentialCommandGroup(
						new AsyncDelay(1.5),
						new ParallelCommandGroup(
								new SetSlidePosition(slide, LinearSlide.HIGH),
								new FlipOut(flipper)
						)
				)
		);
		SelectCommand park = new SelectCommand(
			new HashMap<Object, Command>() {{
				put(1, parkOne);
				put(2, parkTwo);
				put(3, parkThree);
			}},
			() -> signalSide
		);
		ParallelCommandGroup dropForPark = new ParallelCommandGroup(
				new SetSlidePosition(slide, LinearSlide.ONE_CONE),
				new SequentialCommandGroup(
						new AsyncDelay(0.25),
						new OpenClaw(claw),
						new ParallelCommandGroup(
								park,
								new SequentialCommandGroup(
										new AsyncDelay(0.33),
										new FlipIn(flipper)
								)
						)
				)
		);


		SequentialCommandGroup runAuto = new SequentialCommandGroup(
				setUpScoring,
				cycle(5, junctionToConesOne, conesToJunctionOne),
				cycle(4, junctionToConesTwo, conesToJunctionTwo),
				cycle(2, junctionToConesThree, conesToJunctionThree),
				cycle(1, junctionToConesFour, conesToJunctionFour),
				dropForPark,
				new SavePosition(drive::getPoseEstimate)
		);

		ElapsedTime time = new ElapsedTime();
		while (!isStarted() && !isStopRequested()) {
			camera.periodic();
			if (time.time() > 0.5) {
				telemetry.addData("Camera", camera.getConfidences());
				telemetry.update();
				time.reset();
			}
		}

		AutoEndPose.setTimer();
		signalSide = camera.getSide();
		camera.stop();

		schedule(runAuto, new TelemetryUpdate(telemetry));
	}
}
