package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.teamcode.Framework.Commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.Framework.subsystems.AutoDrive;

public class SplineTestingAuto extends CommandOpMode {
	AutoDrive drive;

	@Override
	public void initialize() {
		drive = new AutoDrive(hardwareMap);

		register(drive);

		TrajectoryCommand move = new TrajectoryCommand(drive,
				drive.trajectoryBuilder(new Pose2d(0, 0, 0))
						.splineToSplineHeading(new Pose2d(12, 24, 0), 0)
						.build()
		);

		schedule(move);
	}
}
