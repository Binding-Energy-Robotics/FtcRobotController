package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Framework.Commands.CloseClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.OpenClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.TrajectoryCommand;
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
        this.slide = new LinearSlide(hardwareMap, "slide");

        register(drivetrain, claw, slide);

        // Command Creation
        TrajectoryCommand tc1 = new TrajectoryCommand(drivetrain,
                drivetrain.getDrive().trajectoryBuilder(new Pose2d()).forward(20).build());
        OpenClaw openClaw = new OpenClaw(claw);
        CloseClaw closeClaw = new CloseClaw(claw);

        // Command Scheduling
        schedule(new SequentialCommandGroup(closeClaw, tc1, openClaw));

    }


}
