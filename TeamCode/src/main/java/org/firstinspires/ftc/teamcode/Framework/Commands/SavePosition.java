package org.firstinspires.ftc.teamcode.Framework.Commands;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Framework.Utilities.AutoEndPose;

public class SavePosition extends InstantCommand {
	public interface Pose2dSupplier {
		Pose2d getPose2d();
	}

	private Pose2dSupplier pose;

	public SavePosition(Pose2dSupplier pose) {
		this.pose = pose;
	}

	@Override
	public void initialize() {
		AutoEndPose.setPose(pose.getPose2d());
	}
}
