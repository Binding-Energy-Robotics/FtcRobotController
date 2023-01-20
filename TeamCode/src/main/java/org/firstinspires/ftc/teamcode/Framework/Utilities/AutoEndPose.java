package org.firstinspires.ftc.teamcode.Framework.Utilities;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class AutoEndPose {
	private static Pose2d pose = null;

	public static Pose2d getPose() {
		if (AutoEndPose.pose != null)
			return AutoEndPose.pose;
		return new Pose2d(0, 0, Math.toRadians(90));
	}

	public static void setPose(Pose2d pose) {
		AutoEndPose.pose = pose;
	}

	public static void clearPose() { setPose(new Pose2d(0, 0, 90)); }
}
