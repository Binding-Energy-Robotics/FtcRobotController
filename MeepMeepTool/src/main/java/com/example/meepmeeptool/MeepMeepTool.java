package com.example.meepmeeptool;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTool {
    public static final Pose2d START_POSE = new Pose2d(-63, -36, Math.toRadians(0));
    public static final Pose2d TERMINAL_TURN_POSE_A = new Pose2d(-12, -36, Math.toRadians(0));
    public static final Pose2d TERMINAL_POSE = new Pose2d(-7, -31, Math.toRadians(45));
    public static final Pose2d TERMINAL_TURN_POSE_B = new Pose2d(-12, -36, Math.toRadians(90));
    public static final Pose2d SUBSTATION_TURN_POSE_A = new Pose2d(-59, -36, Math.toRadians(180));
    public static final Pose2d SUBSTATION_TURN_POSE_B = new Pose2d(-59, -24, Math.toRadians(180));
    public static final Pose2d SUBSTATION_POSE = new Pose2d(-61, -12, Math.toRadians(180));
    public static final Pose2d PARK_TRANSITION_POSE_A = new Pose2d(-12, -35, Math.toRadians(90));
    public static final Pose2d PARK_TRANSITION_POSE_B = new Pose2d(-12, -37, Math.toRadians(90));
    public static final Pose2d PARK_POSE_1 = new Pose2d(-12, -12, Math.toRadians(90));
    public static final Pose2d PARK_POSE_2 = new Pose2d(-61, -9, Math.toRadians(45));
    public static final Pose2d PARK_POSE_3 = new Pose2d(-61, -9, Math.toRadians(90));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

//        drive.trajectorySequenceBuilder(START_POSE)
//                .lineTo(TERMINAL_TURN_POSE_A.vec())
//                .splineToSplineHeading(TERMINAL_POSE, TERMINAL_POSE.getHeading())
//                .build()
//        drive.trajectorySequenceBuilder(TERMINAL_POSE)
//                .splineToSplineHeading(TERMINAL_TURN_POSE_B, Math.toRadians(180))
//                .lineTo(SUBSTATION_TURN_POSE.vec())
//                .splineToSplineHeading(SUBSTATION_POSE, SUBSTATION_POSE.getHeading())
//                .waitSeconds(1)
//                .splineToSplineHeading(SUBSTATION_TURN_POSE, Math.toRadians(0))
//                .lineTo(TERMINAL_TURN_POSE_A.vec())
//                .splineToSplineHeading(TERMINAL_POSE, TERMINAL_POSE.getHeading())
//                .build();
//        Trajectory park1 = drive.trajectoryBuilder(TERMINAL_POSE)
//                .splineToSplineHeading(PARK_TRANSITION_POSE_A, Math.toRadians(135))
//                .splineToSplineHeading(PARK_POSE_1, Math.toRadians(90))
//                .build();
//        Trajectory park2 = drive.trajectoryBuilder(TERMINAL_POSE)
//                .splineToSplineHeading(PARK_POSE_2, PARK_POSE_2.getHeading())
//                .build();
//        Trajectory park3 = drive.trajectoryBuilder(TERMINAL_POSE)
//                .splineToSplineHeading(PARK_TRANSITION_POSE_B, Math.toRadians(-120))
//                .splineToSplineHeading(PARK_POSE_3, Math.toRadians(90))
//                .build();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(45, 30, 2.5, Math.toRadians(60), 16.68)
                .followTrajectorySequence(drive ->drive.trajectorySequenceBuilder(TERMINAL_POSE)
                        .splineToLinearHeading(TERMINAL_TURN_POSE_B, Math.toRadians(180))
                        .turn(Math.toRadians(90))
                        .strafeTo(SUBSTATION_TURN_POSE_A.vec())
                        .setTangent(Math.toRadians(90))
                        .strafeTo(SUBSTATION_TURN_POSE_B.vec())
                        .splineToConstantHeading(SUBSTATION_POSE.vec(), SUBSTATION_POSE.getHeading())
                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}