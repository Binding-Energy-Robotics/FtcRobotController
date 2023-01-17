package com.example.meepmeeptool;

import static com.example.meepmeeptool.DriveConstants.MAX_ACCEL;
import static com.example.meepmeeptool.DriveConstants.MAX_ANG_ACCEL;
import static com.example.meepmeeptool.DriveConstants.MAX_ANG_VEL;
import static com.example.meepmeeptool.DriveConstants.MAX_VEL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class LeftAuto {
	public static final Pose2d START_POSE = new Pose2d(-40, -63.5, Math.toRadians(-90));
	public static final Pose2d SCORE_POSE = new Pose2d(-31, -7, Math.toRadians(45));
	public static final Pose2d CONE_POSE = new Pose2d(-57.5, -11.25, Math.toRadians(0));

	public static void main(String[] args) {
		MeepMeep meepMeep = new MeepMeep(600);

		RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
				// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
				.setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, 13)
				.followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(START_POSE)
						.setTangent(Math.toRadians(90))
						.splineToSplineHeading(SCORE_POSE, Math.toRadians(45))
						.build()
				);

		meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot)
				.start();
	}
}
