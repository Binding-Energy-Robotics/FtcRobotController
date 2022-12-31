package org.firstinspires.ftc.teamcode.Framework.Commands.Drive;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.AutoDrive;

public class TrajectoryCommand extends CommandBase {
	AutoDrive drive;
	Trajectory trajectory;

	public TrajectoryCommand(AutoDrive drive, Trajectory trajectory) {
		this.drive = drive;
		this.trajectory = trajectory;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		drive.followTrajectoryAsync(trajectory);
	}

	@Override
	public void execute() {
		drive.update();
	}

	@Override
	public boolean isFinished() {
		return !drive.isBusy();
	}
}
