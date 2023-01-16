package com.example.meepmeeptool;

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
				.setConstraints(45, 30, 2.5, Math.toRadians(60), 16.68)
				.followTrajectorySequence(drive ->drive.trajectorySequenceBuilder(START_POSE)
						.build()
				);

		meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
				.setDarkMode(true)
				.setBackgroundAlpha(0.95f)
				.addEntity(myBot)
				.start();
	}
}
