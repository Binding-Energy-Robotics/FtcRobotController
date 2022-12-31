package org.firstinspires.ftc.teamcode.Framework.Commands.Drive;


import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

public class BlockingTrajectoryCommand extends CommandBase {

    private SampleMecanumDrive drive;
    private Trajectory trajectory;


    public BlockingTrajectoryCommand(AutoDrive drivetrain, Trajectory trajectory){
        this.drive = drivetrain.getDrive();
        this.trajectory = trajectory;
        addRequirements(drivetrain);
    }

    @Override
    public void execute(){
        drive.followTrajectory(trajectory);
    }

    @Override
    public boolean isFinished(){
        return !drive.isBusy();
    }
}
