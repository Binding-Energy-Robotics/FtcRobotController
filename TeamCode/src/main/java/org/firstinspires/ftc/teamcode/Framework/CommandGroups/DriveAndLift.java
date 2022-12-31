package org.firstinspires.ftc.teamcode.Framework.CommandGroups;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.Framework.Commands.Drive.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.Framework.Commands.Slide.SetSlidePosition;
import org.firstinspires.ftc.teamcode.Framework.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

public class DriveAndLift extends ParallelCommandGroup {
	public DriveAndLift(AutoDrive drive, LinearSlide slide, Trajectory trajectory, int height) {
		addCommands(
				new TrajectoryCommand(drive, trajectory), // move robot
				new SetSlidePosition(slide, height) // move slide
		);

		addRequirements(drive, slide);
	}
}
