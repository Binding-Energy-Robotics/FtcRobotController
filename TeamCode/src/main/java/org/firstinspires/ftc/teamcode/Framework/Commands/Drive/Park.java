package org.firstinspires.ftc.teamcode.Framework.Commands.Drive;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.AutoDrive;

import java.util.function.IntSupplier;

public class Park extends CommandBase {
	AutoDrive drive;
	Trajectory park1;
	Trajectory park2;
	Trajectory park3;
	IntSupplier parkPosition;

	public Park(AutoDrive drive, IntSupplier parkPosition,
				Trajectory park1, Trajectory park2, Trajectory park3) {
		this.drive = drive;
		this.park1 = park1;
		this.park2 = park2;
		this.park3 = park3;
		this.parkPosition = parkPosition;
		addRequirements(drive);
	}

	@Override
	public void initialize() {
		Trajectory trajectory;
		int position = parkPosition.getAsInt();

		if (position == 1) {
			trajectory = park1;
		}
		else if (position == 2) {
			trajectory = park2;
		}
		else {
			trajectory = park3;
		}

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
