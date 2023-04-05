package org.firstinspires.ftc.teamcode.Framework.Utilities.DriveSensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kotlin.extensions.geometry.Pose2dExtKt;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.Framework.Utilities.UnscentedKalmanFilter;

public class DriveSystem implements UnscentedKalmanFilter.NonlinearSystem {
	public DriveSystem() {}

	@Override
	public RealVector calculate(RealVector x, RealVector u) {
		Pose2d pose = new Pose2d(x.getEntry(0), x.getEntry(1), x.getEntry(2));
		Vector2d velocity = new Vector2d(x.getEntry(3), x.getEntry(4));
		Vector2d localVelocity = velocity.rotated(-pose.getHeading());
		Twist2d twist = new Twist2d(localVelocity.getX() * 0.05, localVelocity.getY() * 0.05, x.getEntry(5) * 0.05);
		com.arcrobotics.ftclib.geometry.Pose2d next = Pose2dExtKt.exp(new com.arcrobotics.ftclib.geometry.Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getHeading())), twist);
		Vector2d nextVelocity = velocity.rotated(twist.dtheta);
		return new ArrayRealVector(new double[] {
				next.getX(),
				next.getY(),
				next.getHeading(),
				nextVelocity.getX(),
				nextVelocity.getY(),
				x.getEntry(5)
		});
	}
}
