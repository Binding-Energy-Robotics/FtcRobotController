package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.OpenClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.Drive.BlockingTrajectoryCommand;
import org.firstinspires.ftc.teamcode.Framework.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

@Disabled
@Autonomous
public class TestMechAuto extends CommandOpMode {
    private AutoDrive drivetrain;
    private Claw claw;
    private LinearSlide slide;

    @Override
    public void initialize() {

        // Hardware initialization
        this.drivetrain = new AutoDrive(hardwareMap);
        this.claw = new Claw(hardwareMap);
        this.slide = new LinearSlide(hardwareMap, telemetry, true);

        register(drivetrain, claw, slide);

        // Command Creation
        BlockingTrajectoryCommand tc1 = new BlockingTrajectoryCommand(drivetrain,
                drivetrain.getDrive().trajectoryBuilder(new Pose2d()).forward(20).splineTo(new Vector2d(50,50), 30).build());
        OpenClaw openClaw = new OpenClaw(claw);
        CloseClaw closeClaw = new CloseClaw(claw);


        // Command Scheduling
        schedule(new SequentialCommandGroup(closeClaw, tc1, openClaw));


    }


}
