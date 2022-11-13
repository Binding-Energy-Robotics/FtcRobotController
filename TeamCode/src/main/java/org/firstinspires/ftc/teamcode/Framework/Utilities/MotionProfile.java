package org.firstinspires.ftc.teamcode.Framework.Utilities;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Config
public class MotionProfile {
	public static double MAX_VELOCITY = 1000;
	public static double MAX_ACCELERATION = 2000;

	public static double Kv = 0.01;
	public static double Ka = 0;

	private ElapsedTime time;
	private ArrayList<Double> times;
	private ArrayList<Double> velocities;

	public MotionProfile(double currentPos, double currentVel, double targetPos, double targetVel) {
		time = new ElapsedTime();

		times.add(0.0);
		velocities.add(0.0);

		double dx = targetPos - currentPos;

		double accelerationTime = MAX_VELOCITY / MAX_ACCELERATION;

		if (accelerationTime * MAX_VELOCITY < dx) { // trapezoidal profile
			double cruiseDuration = (dx - accelerationTime * MAX_VELOCITY) / MAX_VELOCITY;
			double decelerationTime = accelerationTime + cruiseDuration;
			double endTime = decelerationTime + accelerationTime;

			times.add(accelerationTime);
			velocities.add(MAX_VELOCITY);
			times.add(decelerationTime);
			velocities.add(MAX_VELOCITY);
			times.add(endTime);
			velocities.add(0.0);
		}
		else { // triangular profile
			accelerationTime = Math.sqrt(dx / MAX_ACCELERATION);
			double maxVelocity = MAX_ACCELERATION * accelerationTime;

			times.add(accelerationTime);
			velocities.add(maxVelocity);
			times.add(2.0 * accelerationTime);
			velocities.add(0.0);
		}
	}

	public MotionProfile(double currentPos, double targetPos) {
		this(currentPos, 0, targetPos, 0);
	}
}
