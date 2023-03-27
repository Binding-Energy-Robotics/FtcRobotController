package org.firstinspires.ftc.teamcode.Framework.Utilities.DriveSensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.Framework.Utilities.UnscentedKalmanFilter;
import org.firstinspires.ftc.teamcode.Roadrunner.util.Encoder;


public class DeadWheels implements UnscentedKalmanFilter.NonlinearSensor {
	public static double TICKS_PER_REV = 8192;
	public static double WHEEL_RADIUS = 1; // in
	public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

	public static double X_MULTIPLIER = 0.6866;
	public static double Y_MULTIPLIER = 0.7027;

	private static final Pose2d parallelPose = new Pose2d(0, 0, 0);
	private static final Pose2d perpendicularPose = new Pose2d(0, 0, Math.PI / 2);

	private Encoder parallelEncoder, perpendicularEncoder;

	public DeadWheels(HardwareMap hw) {
		parallelEncoder = new Encoder(
				hw.get(DcMotorEx.class, "frontLeft")
		);
		perpendicularEncoder = new Encoder(
				hw.get(DcMotorEx.class, "frontRight")
		);
	}

	public static double ticksToInches(double ticks) {
		return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
	}

	public double[] getEncoderVelocities() {
		return new double[] {
				ticksToInches(parallelEncoder.getCorrectedVelocity()) * X_MULTIPLIER,
				ticksToInches(perpendicularEncoder.getCorrectedVelocity()) * Y_MULTIPLIER
		};
	}

	@Override
	public RealVector measure(RealVector x, RealVector u) {
		double heading = x.getEntry(2);
		double xVelocity = x.getEntry(3);
		double yVelocity = x.getEntry(4);
		double headingVelocity = x.getEntry(5);

		Vector2d velocity = new Vector2d(xVelocity, yVelocity);
		velocity = velocity.rotated(-heading);
		double forwardVelocity = velocity.getX();
		double leftVelocity = velocity.getY();

		forwardVelocity -= headingVelocity * parallelPose.getY();
		leftVelocity += headingVelocity * perpendicularPose.getX();

		return new ArrayRealVector(new double[] {
				forwardVelocity, leftVelocity
		});
	}

	@Override
	public RealVector getData() {
		return new ArrayRealVector(getEncoderVelocities());
	}
}
