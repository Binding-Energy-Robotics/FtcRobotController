package com.example.meepmeeptool;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class A2Auto {
	public static final Pose2d START_POSE = new Pose2d(-36, 61, Math.toRadians(-90));
	public static final Pose2d TERMINAL_TURN_A = new Pose2d(-36, 13, Math.toRadians(-90));
	public static final Pose2d TERMINAL_TURN_B = new Pose2d(-30, 6, Math.toRadians(-45));
	public static final Pose2d TERMINAL_POSE = new Pose2d(-28, 4, Math.toRadians(-45));
	public static final Pose2d TERMINAL_TURN_C = new Pose2d(-37, 12, Math.toRadians(0));
	public static final Pose2d CONE_STACK = new Pose2d(-61, 12, Math.toRadians(0));
	public static final Pose2d PARK_1 = new Pose2d(-60, 12, Math.toRadians(0));
	public static final Pose2d PARK_2 = new Pose2d(-36, 12, Math.toRadians(0));
	public static final Pose2d PARK_3 = new Pose2d(-12, 12, Math.toRadians(0));

	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(600);

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(45, 30, 2.5, Math.toRadians(60), 16.68)
				.followTrajectorySequence(drive ->drive.trajectorySequenceBuilder(START_POSE)
						.strafeTo(TERMINAL_TURN_A.vec())
						.splineToSplineHeading(TERMINAL_TURN_B, Math.toRadians(-45))
						.strafeTo(TERMINAL_POSE.vec())
						.waitSeconds(0.5) // zeroth cone --------------------
						.strafeTo(TERMINAL_TURN_B.vec())
						.splineToSplineHeading(TERMINAL_TURN_C, Math.toRadians(180))
						.strafeTo(CONE_STACK.vec())
						.waitSeconds(1)
						.strafeTo(TERMINAL_TURN_C.vec())
						.splineToSplineHeading(TERMINAL_TURN_B, Math.toRadians(-45))
						.strafeTo(TERMINAL_POSE.vec())
						.waitSeconds(0.5) // first cone --------------------
						.strafeTo(TERMINAL_TURN_B.vec())
						.splineToSplineHeading(TERMINAL_TURN_C, Math.toRadians(180))
						.strafeTo(CONE_STACK.vec())
						.waitSeconds(1)
						.strafeTo(TERMINAL_TURN_C.vec())
						.splineToSplineHeading(TERMINAL_TURN_B, Math.toRadians(-45))
						.strafeTo(TERMINAL_POSE.vec())
						.waitSeconds(0.5) // second cone --------------------
						.strafeTo(TERMINAL_TURN_B.vec())
						.splineToSplineHeading(TERMINAL_TURN_C, Math.toRadians(180))
						.strafeTo(CONE_STACK.vec())
						.waitSeconds(1)
						.strafeTo(TERMINAL_TURN_C.vec())
						.splineToSplineHeading(TERMINAL_TURN_B, Math.toRadians(-45))
						.strafeTo(TERMINAL_POSE.vec())
						.waitSeconds(0.5) // third cone --------------------
						.strafeTo(TERMINAL_TURN_B.vec())
						.splineToSplineHeading(TERMINAL_TURN_C, Math.toRadians(180))
						.strafeTo(PARK_3.vec())
						.build()
				);

		meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot)
				.start();
	}
}
