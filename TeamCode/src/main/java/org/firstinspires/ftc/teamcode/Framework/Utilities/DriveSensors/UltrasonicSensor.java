package org.firstinspires.ftc.teamcode.Framework.Utilities.DriveSensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.Framework.Utilities.UnscentedKalmanFilter;

public class UltrasonicSensor implements UnscentedKalmanFilter.NonlinearSensor {
	private static final double SENSOR_VOLTAGE = 5;
	private AnalogInput distanceSensor;
	private Pose2d sensorPose;

	public UltrasonicSensor(HardwareMap hw, String name, Pose2d sensorPose) {
		distanceSensor = hw.get(AnalogInput.class, name);
		this.sensorPose = sensorPose;
	}

	@Override
	public RealVector measure(RealVector x, RealVector u) {
		Pose2d robotPose = new Pose2d(x.getEntry(0), x.getEntry(1), x.getEntry(2));
		Pose2d currentSensorPose = sensorPose.plus(robotPose);
		Vector2d currentSensorLocation = currentSensorPose.vec();
		Vector2d currentSensorHeadingVector = currentSensorPose.headingVec();

		Vector2d corner = new Vector2d(
				Math.copySign(72, currentSensorHeadingVector.getX()),
				Math.copySign(72, currentSensorHeadingVector.getY())
		);
		Vector2d sensorToCorner = corner.minus(currentSensorLocation);
		double sensorAngle = currentSensorHeadingVector.angle();
		double cornerAngle = sensorToCorner.angle();

		// get angle from x axis
		sensorAngle = Math.PI / 2 - Math.abs(Math.abs(sensorAngle) - Math.PI / 2);
		cornerAngle = Math.PI / 2 - Math.abs(Math.abs(cornerAngle) - Math.PI / 2);

		double distance = 0;
		if (cornerAngle >= sensorAngle) { // read from x
			distance = sensorToCorner.getX() / currentSensorHeadingVector.getX();
		}
		else { // read from y
			distance = sensorToCorner.getY() / currentSensorHeadingVector.getY();
		}

		return new ArrayRealVector(new double[] { distance });
	}

	@Override
	public RealVector getData() {
		double distance = getDistance();
		return new ArrayRealVector(new double[] { distance });
	}

	//  distance = [V_observed / ((Vcc/1024) * 6)] - 300
	public double getDistance() { // returns distance in inches
		return (distanceSensor.getVoltage() / ((SENSOR_VOLTAGE / 1024) * 6) - 300) / 25.4;
	}
}
