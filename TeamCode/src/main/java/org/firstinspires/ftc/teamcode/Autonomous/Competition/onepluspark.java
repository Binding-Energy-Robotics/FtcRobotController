package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
import org.firstinspires.ftc.teamcode.Framework.Utilities.AutoEndPose;
import org.firstinspires.ftc.teamcode.Framework.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Camera;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

import java.util.HashMap;

@Config
@Autonomous(preselectTeleOp = "MainTeleop")
public class onepluspark extends CommandOpMode {
//    public static Pose2d START_POSE = new Pose2d(-31, -64.5, Math.toRadians(-90));
//    public static Pose2d START_POSE_A = new Pose2d(-34, -28, Math.toRadians(-90));
//    public static Pose2d START_POSE_B = new Pose2d(-34, -24, Math.toRadians(-90));
//
//    public static final Pose2d START_POSE_C = new Pose2d(-39, -8, Math.toRadians(-30));
//
//    public static Pose2d SCORE_POSE_ZERO = new Pose2d(-30.4, -16, Math.toRadians(-45));
 //   public static Pose2d SCORE_POSE_ONE = new Pose2d(-30.4, -17, Math.toRadians(-45));
   // public static Pose2d SCORE_POSE_TWO = new Pose2d(-30, -17, Math.toRadians(-45));
   // public static Pose2d SCORE_POSE_THREE = new Pose2d(-29, -17, Math.toRadians(-45));
   // public static Pose2d SCORE_POSE_FOUR = new Pose2d(-29, -16, Math.toRadians(-45));
   // public static Pose2d SCORE_POSE_FIVE = new Pose2d(-28.5, -15, Math.toRadians(-45));

//    public static double posePickupYoffset_sus = -2;
 //   public static Pose2d CONE_POSE_ONE = new Pose2d(-57 + posePickupYoffset_sus , -12.5, Math.toRadians(0));
  //  public static Pose2d CONE_POSE_TWO = new Pose2d(-57+ posePickupYoffset_sus, -11.5, Math.toRadians(0));
  //  public static Pose2d CONE_POSE_THREE = new Pose2d(-57+ posePickupYoffset_sus, -11, Math.toRadians(0));
 //   public static Pose2d CONE_POSE_FOUR = new Pose2d(-57+ posePickupYoffset_sus, -10.4, Math.toRadians(0));
  //  public static Pose2d CONE_POSE_FIVE = new Pose2d(-57+ posePickupYoffset_sus, -10.2, Math.toRadians(0));

//    public static Pose2d ZONE_ONE = new Pose2d(-58, -20, Math.toRadians(0));
//    public static Pose2d ZONE_TWO = new Pose2d(-36, -20, Math.toRadians(90));
//
//    public static Pose2d ZONE_THREE = new Pose2d(-13, -20, Math.toRadians(90));

    Telemetry telemetry;

    AutoDrive drive;
    LinearSlide slide;
    Flipper flipper;
    Claw claw;
    Camera camera;

    public static int signalSide = 1;

//    private Command cycle(int coneHeight, Trajectory junctionToCones, Trajectory conesToJunction) {
//        return new SequentialCommandGroup(
//                new ParallelCommandGroup(
//                        //  new SetSlidePosition(slide, LinearSlide.CONE_STACK[coneHeight]),
//                        new SetSlidePosition(slide, LinearSlide.ONE_CONE, 1),
//                        new SequentialCommandGroup(
//                                new AsyncDelay(0.09),
//                                new OpenClaw(claw),
//                                new AsyncDelay(0.1),
//                                new SetSlidePosition(slide, LinearSlide.CONE_STACK[coneHeight], 2).alongWith(
//                                        new AsyncDelay(0.25).andThen(
//                                                new FlipIn(flipper)
//                                        )
//                                )
//                        ),
//                        new SequentialCommandGroup(
//                                new AsyncDelay(0.09),
//                                new TrajectoryCommand(drive, junctionToCones)
//                        )
//                ),
//                new CloseClaw(claw),
//                new AsyncDelay(0.2),
//                new ParallelCommandGroup(
//                        new SetSlidePosition(slide, LinearSlide.MEDIUM),
//                        new SequentialCommandGroup(
//                                new AsyncDelay(0.25),
//                                new ParallelCommandGroup(
//                                        new TrajectoryCommand(drive, conesToJunction),
//                                        new SequentialCommandGroup(
//                                                new AsyncDelay(0.08),
//                                                //    new ParallelCommandGroup(
//                                                //          new SetSlidePosition(slide, LinearSlide.TWO_CONE),
//                                                //  new SequentialCommandGroup(
//                                                //    new AsyncDelay(0.02),
//                                                new ParallelCommandGroup(
//                                                             new SetSlidePosition(slide, LinearSlide.MEDIUM),
//                                                        new SequentialCommandGroup(
//                                                                new AsyncDelay(0.07),
//                                                                new FlipOut(flipper)
//                                                        )
//                                                )
//                                        )
//                                        //)
//                                )
//                        )
//                )
//        );
//        //);
//    }

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(super.telemetry,
                FtcDashboard.getInstance().getTelemetry());

        drive = new AutoDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(-90)));
        slide = new LinearSlide(hardwareMap, telemetry);
        flipper = new Flipper(hardwareMap, telemetry);
        claw = new Claw(hardwareMap);
        claw.open();
        camera = new Camera(hardwareMap);

        Pose2d SECOND_POSE = new Pose2d(-6, 32, Math.toRadians(-90));
        Pose2d THIRD_POSE = new Pose2d(-7, 27, Math.toRadians(135));
        Pose2d FOURTH_POSE = new Pose2d(-13, 34, Math.toRadians(135));

        Pose2d FIFTH_POSE = new Pose2d(-3, 29, Math.toRadians(0));

        Pose2d ONE = new Pose2d(-28, 29, Math.toRadians(0));
        Pose2d THREE = new Pose2d(22, 29, Math.toRadians(0));

        Command push = new TrajectoryCommand(drive,
                drive.trajectoryBuilder(new Pose2d(0, 0, Math.toRadians(-90)))
                        .lineTo(SECOND_POSE.vec())
                        .build()
        ).andThen(
                new TrajectoryCommand(drive,
                        drive.trajectoryBuilder(SECOND_POSE)
                                .lineToLinearHeading(THIRD_POSE)
                                .build()
                )
        ).andThen(
                new TrajectoryCommand(drive,
                        drive.trajectoryBuilder(THIRD_POSE)
                                .lineTo(FOURTH_POSE.vec())
                                .build()
                )
        );

        Command ready = new TrajectoryCommand(drive,
                drive.trajectoryBuilder(FOURTH_POSE)
                        .lineToLinearHeading(FIFTH_POSE)
                        .build()
        );

        Command parkOne = new TrajectoryCommand(drive,
                drive.trajectoryBuilder(FIFTH_POSE)
                        .lineTo(ONE.vec())
                        .build()
        );
        Command parkTwo = new AsyncDelay(0);
        Command parkThree = new TrajectoryCommand(drive,
                drive.trajectoryBuilder(FIFTH_POSE)
                        .lineTo(THREE.vec())
                        .build()
        );

        Command place = new ParallelCommandGroup(
                push,
                new CloseClaw(claw),
                new AsyncDelay(0.5).andThen(
                        new FlipOut(flipper).alongWith(
                                new SetSlidePosition(slide, LinearSlide.MEDIUM, 2)
                        )
                )
        ).andThen(
                new SetSlidePosition(slide, LinearSlide.GROUND, 2).alongWith(
                        new AsyncDelay(0.5).andThen(
                                new OpenClaw(claw).andThen(
                                        new AsyncDelay(0.25).andThen(
                                                new FlipIn(flipper)
                                        )
                                )
                        )
                )
        ).andThen(
                ready
        );


  //      Trajectory junctionToConesOne = drive.trajectoryBuilder(SCORE_POSE_ZERO, true)
   //             .splineTo(CONE_POSE_ONE.vec(), Math.toRadians(180))
   //             .build();
  //      Trajectory conesToJunctionOne = drive.trajectoryBuilder(
   //                     junctionToConesOne.end(), Math.toRadians(0))
   //             .splineToSplineHeading(SCORE_POSE_ONE, SCORE_POSE_ONE.getHeading())
  //              .build();

//        Trajectory junctionToConesTwo = drive.trajectoryBuilder(conesToJunctionOne.end(), true)
//                .splineTo(CONE_POSE_TWO.vec(), Math.toRadians(180))
//                .build();
//        Trajectory conesToJunctionTwo = drive.trajectoryBuilder(
//                        junctionToConesTwo.end(), Math.toRadians(0))
//                .splineToSplineHeading(SCORE_POSE_TWO, SCORE_POSE_TWO.getHeading())
//                .build();
//
//        Trajectory junctionToConesThree = drive.trajectoryBuilder(conesToJunctionTwo.end(), true)
//                .splineTo(CONE_POSE_THREE.vec(), Math.toRadians(180))
//                .build();
//        Trajectory conesToJunctionThree = drive.trajectoryBuilder(
//                        junctionToConesThree.end(), Math.toRadians(0))
//                .splineToSplineHeading(SCORE_POSE_THREE, SCORE_POSE_THREE.getHeading())
//                .build();
//
//        Trajectory junctionToConesFour = drive.trajectoryBuilder(conesToJunctionThree.end(), true)
//                .splineTo(CONE_POSE_FOUR.vec(), Math.toRadians(180))
//                .build();
//        Trajectory conesToJunctionFour = drive.trajectoryBuilder(
//                        junctionToConesFour.end(), Math.toRadians(0))
//                .splineToSplineHeading(SCORE_POSE_FOUR, SCORE_POSE_FOUR.getHeading())
//                .build();
//
//        Trajectory junctionToConesFive = drive.trajectoryBuilder(conesToJunctionFour.end(), true)
//                .splineTo(CONE_POSE_FIVE.vec(), Math.toRadians(180))
//                .build();
//        Trajectory conesToJunctionFive = drive.trajectoryBuilder(
//                        junctionToConesFive.end(), Math.toRadians(0))
//                .splineToSplineHeading(SCORE_POSE_FIVE, SCORE_POSE_FIVE.getHeading())
//                .build();
//

//        TrajectoryCommand startToJunction = new TrajectoryCommand(drive,
//                drive.trajectoryBuilder(START_POSE, Math.toRadians(90))
//                        .splineToSplineHeading(START_POSE_A, Math.toRadians(90))
//                        .splineToSplineHeading(START_POSE_B, Math.toRadians(90))
//                        .splineToSplineHeading(START_POSE_C, Math.toRadians(100))
//                        .splineToConstantHeading(SCORE_POSE_ZERO.vec(), Math.toRadians(-45))
//                        .build()
//        );
//        TrajectoryCommand parkOne = new TrajectoryCommand(drive,
//                drive.trajectoryBuilder(SCORE_POSE_ZERO, true)
//                        .splineToSplineHeading(ZONE_ONE, Math.toRadians(180))
//                        .build()
//        );
//        TrajectoryCommand parkTwo = new TrajectoryCommand(drive,
//                drive.trajectoryBuilder(SCORE_POSE_ZERO)
//                        .lineToLinearHeading(ZONE_TWO)
//                        .build()
//        );
//        TrajectoryCommand parkThree = new TrajectoryCommand(drive,
//                drive.trajectoryBuilder(SCORE_POSE_ZERO, Math.toRadians(0))
//                        .splineToSplineHeading(ZONE_TWO, Math.toRadians(0))
//                        .splineToSplineHeading(ZONE_THREE, Math.toRadians(90))
//                        .build()
//        );


//        ParallelCommandGroup setUpScoring = new ParallelCommandGroup(
//                startToJunction,
//                new CloseClaw(claw),
//                new SequentialCommandGroup(
//                        new AsyncDelay(.15),
//                        new ParallelCommandGroup(
//                                new SetSlidePosition(slide, LinearSlide.MEDIUM),
//                                new FlipOut(flipper)
//                        )
//                )
//        );
        SelectCommand park = new SelectCommand(
                new HashMap<Object, Command>() {{
                    put(1, parkOne);
                    put(2, parkTwo);
                    put(3, parkThree);
                }},
                () -> signalSide
        );
//        ParallelCommandGroup dropForPark = new ParallelCommandGroup(
//                new SetSlidePosition(slide, LinearSlide.ONE_CONE),
//                new SequentialCommandGroup(
//                        new AsyncDelay(0.1),
//                        new OpenClaw(claw),
//                        new ParallelCommandGroup(
//                                park,
//                                new SequentialCommandGroup(
//                                        new AsyncDelay(.5),
//                                        new FlipIn(flipper)
//                                )
//                        )
//                )
//        );


        SequentialCommandGroup runAuto = new SequentialCommandGroup(
      //          setUpScoring,
           //     cycle(5, junctionToConesOne, conesToJunctionOne),
              //  cycle(4, junctionToConesTwo, conesToJunctionTwo),
              //  cycle(3, junctionToConesThree, conesToJunctionThree),
            //    cycle(2, junctionToConesFour, conesToJunctionFour),
            //    cycle(1, junctionToConesFive, conesToJunctionFive),
//                dropForPark,
                place,
                park,
                new SavePosition(drive::getPoseEstimate)
        );

        register(drive, slide, flipper, claw);

        ElapsedTime time = new ElapsedTime();
        while (!isStarted()) {
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

        schedule(runAuto);
    }
}