package org.firstinspires.ftc.teamcode.Framework.CommandGroups;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.OpenClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.Drive.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.Framework.Commands.Slide.SetSlidePosition;
import org.firstinspires.ftc.teamcode.Framework.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;

public class ReleaseThenDrive extends SequentialCommandGroup {
	public ReleaseThenDrive(AutoDrive drive, Claw claw, Trajectory trajectory) {
		addCommands(
				new OpenClaw(claw), // open claw
				new TrajectoryCommand(drive, trajectory) // move robot
		);

		addRequirements(drive, claw);
	}
}
