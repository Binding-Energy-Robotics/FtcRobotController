package com.example.meepmeeptool;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;

public class UnitTesting {
	static Pose2d sensorPose = new Pose2d(-3, 6, Math.PI / 2);

	public static void main(String[] args) {
		RealVector x = new ArrayRealVector(new double[] {
				5, 27, Math.PI
		});
		RealVector measurement = measure(x, null);
		System.out.println(measurement);
	}

	public static RealVector measure(RealVector x, RealVector u) {
		Pose2d robotPose = new Pose2d(x.getEntry(0), x.getEntry(1), x.getEntry(2));
		Vector2d relativeSensorPose = sensorPose.vec().rotated(robotPose.getHeading());
		Pose2d currentSensorPose = robotPose.plus(
				new Pose2d(relativeSensorPose, sensorPose.getHeading())
		);
		System.out.println(currentSensorPose);
		Vector2d currentSensorLocation = currentSensorPose.vec();
		Vector2d currentSensorHeadingVector = currentSensorPose.headingVec();
		System.out.println(currentSensorHeadingVector);

		Vector2d corner = new Vector2d(
				Math.copySign(72, currentSensorHeadingVector.getX()),
				Math.copySign(72, currentSensorHeadingVector.getY())
		);
		System.out.println(corner);
		Vector2d sensorToCorner = corner.minus(currentSensorLocation);
		double sensorAngle = currentSensorHeadingVector.angle();
		double cornerAngle = sensorToCorner.angle();

		System.out.println(Math.toDegrees(sensorAngle));
		System.out.println(Math.toDegrees(cornerAngle));

		// get angle from x axis
		sensorAngle = Math.PI / 2 - Math.abs(Math.abs(sensorAngle - Math.PI) - Math.PI / 2);
		cornerAngle = Math.PI / 2 - Math.abs(Math.abs(cornerAngle - Math.PI) - Math.PI / 2);

		System.out.println(Math.toDegrees(sensorAngle));
		System.out.println(Math.toDegrees(cornerAngle));

		double distance;
		if (cornerAngle >= sensorAngle) { // read from x
			System.out.println("x");
			distance = sensorToCorner.getX() / currentSensorHeadingVector.getX();
		}
		else { // read from y
			System.out.println("y");
			distance = sensorToCorner.getY() / currentSensorHeadingVector.getY();
		}

		return new ArrayRealVector(new double[] { distance });
	}
}
