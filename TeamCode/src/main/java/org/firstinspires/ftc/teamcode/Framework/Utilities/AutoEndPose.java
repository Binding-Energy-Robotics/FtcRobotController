package org.firstinspires.ftc.teamcode.Framework.Utilities;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoEndPose {
	public static final double AUTO_START = 0;
	public static final double AUTO_END = 30;
	public static final double TELEOP_START = 38;
	public static final double TELEOP_END = 158;
	public static final double ENDGAME_START = 128;

	private static Pose2d pose = new Pose2d(0, 0, Math.toRadians(90));
	private static ElapsedTime timer = null;
	private static int slidePosition;

	public static Pose2d getPose() { return AutoEndPose.pose; }

	public static void setPose(Pose2d pose) { AutoEndPose.pose = pose; }

	public static void clearPose() { setPose(new Pose2d(0, 0, Math.toRadians(90))); }

	public static ElapsedTime getTimer() {
		if (timer == null) {
			timer = new ElapsedTime(System.nanoTime() -
					(long)(TELEOP_START * ElapsedTime.SECOND_IN_NANO));
		}
		return timer;
	}

	public static void clearTimer() { timer = null; }

	public static void setTimer() { timer = new ElapsedTime(); }

	public static int getSlidePosition() { return AutoEndPose.slidePosition; }

	public static void setSlidePosition(int slidePosition) {
		AutoEndPose.slidePosition = slidePosition;
	}

	public static void clearSlidePosition() { setSlidePosition(0); }
}
