package org.firstinspires.ftc.teamcode.Roadrunner.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.Framework.Utilities.DriveSensors.DeadWheels;
import org.firstinspires.ftc.teamcode.Framework.Utilities.DriveSensors.DriveSystem;
import org.firstinspires.ftc.teamcode.Framework.Utilities.DriveSensors.Gyroscope;
import org.firstinspires.ftc.teamcode.Framework.Utilities.DriveSensors.UltrasonicGroup;
import org.firstinspires.ftc.teamcode.Framework.Utilities.DriveSensors.UltrasonicSensor;
import org.firstinspires.ftc.teamcode.Framework.Utilities.UnscentedKalmanFilter;

public class UnscentedLocalizer implements Localizer {
	private UnscentedKalmanFilter filter;

	private DeadWheels deadWheels;
	private Gyroscope gyro;
	private UltrasonicGroup distanceSensors;

	public UnscentedLocalizer(HardwareMap hw, Pose2d initialPose) {
		filter = new UnscentedKalmanFilter(
				new DriveSystem(),
				new Array2DRowRealMatrix(new double[][] {
						{ .2, 0, 0, 0, 0, 0 },
						{ 0, .2, 0, 0, 0, 0 },
						{ 0, 0, .02, 0, 0, 0 },
						{ 0, 0, 0, .5, 0, 0 },
						{ 0, 0, 0, 0, .5, 0 },
						{ 0, 0, 0, 0, 0, .05 }
				}),
				2,
				new ArrayRealVector(new double[] {
						initialPose.getX(),
						initialPose.getY(),
						initialPose.getHeading(),
						0,
						0,
						0
				}),
				new Array2DRowRealMatrix(new double[][]{
						{ 0.25, 0, 0, 0, 0, 0 },
						{ 0, 0.25, 0, 0, 0, 0 },
						{ 0, 0, 0.025, 0, 0, 0 },
						{ 0, 0, 0, 0, 0, 0 },
						{ 0, 0, 0, 0, 0, 0 },
						{ 0, 0, 0, 0, 0, 0 }
				})
		);

		deadWheels = new DeadWheels(hw);
		gyro = new Gyroscope(hw);
		distanceSensors = new UltrasonicGroup(hw);
	}

	public void update(RealVector u) {
		distanceSensors.trigger();

		filter.predict(u);

		filter.correct(u, gyro, Gyroscope.R);
		filter.correct(u, deadWheels, DeadWheels.R);

		distanceSensors.read();

		UltrasonicSensor[] sensors = distanceSensors.getNewData();

		for (UltrasonicSensor sensor : sensors) {
			filter.correct(u, sensor, UltrasonicGroup.R);
		}
	}

	@NonNull
	@Override
	public Pose2d getPoseEstimate() {
		return new Pose2d(
				filter.getXHat().getEntry(0),
				filter.getXHat().getEntry(1),
				filter.getXHat().getEntry(2)
		);
	}

	@Override
	public void setPoseEstimate(@NonNull Pose2d pose2d) {
		filter.setXHat(pose2d);
	}

	@Nullable
	@Override
	public Pose2d getPoseVelocity() {
		return new Pose2d(
				filter.getXHat().getEntry(3),
				filter.getXHat().getEntry(4),
				filter.getXHat().getEntry(5)
		);
	}

	@Override
	public void update() {
		update(new ArrayRealVector(new double[] { 0, 0, 0 }));
	}
}
