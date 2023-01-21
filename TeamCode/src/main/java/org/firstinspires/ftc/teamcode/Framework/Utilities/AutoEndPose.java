package org.firstinspires.ftc.teamcode.Framework.Utilities;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class AutoEndPose {
	private static Pose2d pose = new Pose2d(0, 0, Math.toRadians(90));
	private static int slidePosition;

	public static Pose2d getPose() { return AutoEndPose.pose; }

	public static void setPose(Pose2d pose) { AutoEndPose.pose = pose; }

	public static void clearPose() { setPose(new Pose2d(0, 0, Math.toRadians(90))); }

	public static int getSlidePosition() { return AutoEndPose.slidePosition; }

	public static void setSlidePosition(int slidePosition) {
		AutoEndPose.slidePosition = slidePosition;
	}

	public static void clearSlidePosition() { setSlidePosition(0); }
}
