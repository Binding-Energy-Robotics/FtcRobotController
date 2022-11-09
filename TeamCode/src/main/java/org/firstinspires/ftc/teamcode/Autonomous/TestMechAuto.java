package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.OpenClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.Drive.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.Framework.Commands.Slide.SetAbsoluteSlidestate;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideState;
import org.firstinspires.ftc.teamcode.Framework.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

@Autonomous
public class TestMechAuto extends CommandOpMode {
    private AutoDrive drivetrain;
    private Claw claw;
    private LinearSlide slide;

    @Override
    public void initialize() {

        // Hardware initialization
        this.drivetrain = new AutoDrive(hardwareMap);
        this.claw = new Claw(hardwareMap, "claw");
        this.slide = new LinearSlide(hardwareMap, "slide", telemetry);

        register(drivetrain, claw, slide);

        // Command Creation
        TrajectoryCommand tc1 = new TrajectoryCommand(drivetrain,
                drivetrain.getDrive().trajectoryBuilder(new Pose2d()).forward(20).splineTo(new Vector2d(50,50), 30).build());
        OpenClaw openClaw = new OpenClaw(claw);
        CloseClaw closeClaw = new CloseClaw(claw);
        SetAbsoluteSlidestate slideDown = new SetAbsoluteSlidestate(slide, SlideState.DOWN);
        SetAbsoluteSlidestate slideLow = new SetAbsoluteSlidestate(slide, SlideState.LOW);
        SetAbsoluteSlidestate slideMed = new SetAbsoluteSlidestate(slide, SlideState.MEDIUM);
        SetAbsoluteSlidestate slideHigh = new SetAbsoluteSlidestate(slide, SlideState.HIGH);


        // Command Scheduling
        schedule(new SequentialCommandGroup(closeClaw, tc1, openClaw));


    }


}
