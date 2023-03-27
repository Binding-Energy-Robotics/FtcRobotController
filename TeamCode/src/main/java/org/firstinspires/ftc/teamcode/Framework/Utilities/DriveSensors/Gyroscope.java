package org.firstinspires.ftc.teamcode.Framework.Utilities.DriveSensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.Framework.Utilities.UnscentedKalmanFilter;

public class Gyroscope implements UnscentedKalmanFilter.NonlinearSensor {
	private BNO055IMU imu;

	public Gyroscope(HardwareMap hw) {
		imu = hw.get(BNO055IMU.class,"imu");
		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
		imu.initialize(parameters);
	}

	@Override
	public RealVector measure(RealVector x, RealVector u) {
		double heading = x.getEntry(2);
		double headingVelocity = x.getEntry(5);

		return new ArrayRealVector(new double[] {
				heading, headingVelocity
		});
	}

	@Override
	public RealVector getData() {
		double heading = imu.getAngularOrientation().firstAngle;
		double headingVelocity = -imu.getAngularVelocity().yRotationRate;
		return new ArrayRealVector(new double[] {
				heading, headingVelocity
		});
	}
}
