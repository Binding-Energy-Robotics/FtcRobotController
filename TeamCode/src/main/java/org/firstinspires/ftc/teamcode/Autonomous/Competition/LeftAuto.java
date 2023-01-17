package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SelectCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.Commands.Drive.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.Framework.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Camera;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

import java.util.HashMap;

public class LeftAuto extends CommandOpMode {
	public static final Pose2d START_POSE = new Pose2d(-31, -63.5, Math.toRadians(-90));
	public static final Pose2d START_POSE_A = new Pose2d(-34, -48, Math.toRadians(120));
	public static final Pose2d START_POSE_B = new Pose2d(-36, -24, Math.toRadians(70));
	public static final Pose2d SCORE_POSE = new Pose2d(-31, -7, Math.toRadians(45));
	public static final Pose2d CONE_POSE = new Pose2d(-57.5, -11.25, Math.toRadians(0));
	public static final Pose2d ZONE_ONE = new Pose2d(-59, -12, Math.toRadians(0));
	public static final Pose2d ZONE_TWO = new Pose2d(-36, -12, Math.toRadians(90));
	public static final Pose2d ZONE_THREE = new Pose2d(-12, -12, Math.toRadians(90));

	Telemetry telemetry;

	AutoDrive drive;
	LinearSlide slide;
	Flipper flipper;
	Claw claw;
	Camera camera;

	public static int signalSide;

	@Override
	public void initialize() {
		telemetry = new MultipleTelemetry(super.telemetry,
				FtcDashboard.getInstance().getTelemetry());

		drive = new AutoDrive(hardwareMap);
		slide = new LinearSlide(hardwareMap, telemetry);
		flipper = new Flipper(hardwareMap, telemetry);
		claw = new Claw(hardwareMap);
//		camera = new Camera(hardwareMap);


		TrajectoryCommand startToJunction = new TrajectoryCommand(drive,
				drive.trajectoryBuilder(START_POSE, Math.toRadians(90))
				.splineTo(START_POSE_A.vec(), START_POSE_A.getHeading())
				.splineToSplineHeading(START_POSE_B, Math.toRadians(75))
				.splineToSplineHeading(SCORE_POSE, SCORE_POSE.getHeading())
				.build()
		);
		TrajectoryCommand junctionToCones = new TrajectoryCommand(drive,
				drive.trajectoryBuilder(SCORE_POSE, true)
				.splineTo(CONE_POSE.vec(), Math.toRadians(180))
				.build()
		);
		TrajectoryCommand conesToJunction = new TrajectoryCommand(drive,
				drive.trajectoryBuilder(CONE_POSE, Math.toRadians(0))
				.splineTo(SCORE_POSE.vec(), SCORE_POSE.getHeading())
				.build()
		);

		TrajectoryCommand parkOne = new TrajectoryCommand(drive,
				drive.trajectoryBuilder(SCORE_POSE, true)
				.splineToSplineHeading(ZONE_ONE, Math.toRadians(180))
				.build()
		);
		TrajectoryCommand parkTwo = new TrajectoryCommand(drive,
				drive.trajectoryBuilder(SCORE_POSE)
				.lineToLinearHeading(ZONE_TWO)
				.build()
		);
		TrajectoryCommand parkThree = new TrajectoryCommand(drive,
				drive.trajectoryBuilder(SCORE_POSE, Math.toRadians(-45))
				.splineToSplineHeading(ZONE_THREE, Math.toRadians(0))
				.build()
		);


		SelectCommand park = new SelectCommand(
			new HashMap<Object, Command>() {{
				put(1, parkOne);
				put(2, parkTwo);
				put(3, parkThree);
			}},
			() -> signalSide
		);
	}
}
