package org.firstinspires.ftc.teamcode.Framework.Utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.Util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SlideController {
	public static double Kp = 0;
	public static double Ki = 0;
	public static double Kd = 0;
	private PIDCoefficients coefficients;

	// tuned by Alex Prichard on 14 Dec 2022
	public static int[] SLIDE_SEGMENTS = new int[] { 800, 1600, 2400, 3200 };
	public static double[] GRAVITY_FEEDFORWARDS = new double[] { 0.001, 0.001, 0.05, 0.05 };

	public static double Kv = 0.0004;
	public static double Ka = 0;

	public static double MAX_V = 5000;
	public static double MAX_A = 10000;
	public static double MAX_J = 0;

	private double prevSP = 0;
	public static double SP = 0;

	private double prevPv = 0;
	private double prevVel = 0;

	private double power = 0;

	private PIDFController controller;

	private MotionProfile motionProfile;
	private ElapsedTime elapsedTime;

	private long prevTime;
	private FtcDashboard dash;
	private Telemetry telemetry;

	public SlideController() {
		coefficients = new PIDCoefficients(Kp, Ki, Kd);
		controller = new PIDFController(coefficients, 0, 0);
		prevTime = System.nanoTime();
		dash = FtcDashboard.getInstance();
		telemetry = dash.getTelemetry();
		motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
				new MotionState(0, 0, 0),
				new MotionState(0, 0, 0),
				MAX_V,
				MAX_A,
				MAX_J
		);
		elapsedTime = new ElapsedTime();
	}

	public double getKg(int position) {
		for (int i = 0; i < SLIDE_SEGMENTS.length; i++) {
			if (position < SLIDE_SEGMENTS[i]) {
				return GRAVITY_FEEDFORWARDS[i];
			}
		}
		return 0;
	}

	public double getPower(int Pv) {
		long time = System.nanoTime();
		double dt = (time - prevTime) * 1.0e-9;

		if (dt > 1e-2) {
			double vel = (Pv - prevPv) / dt;
			prevVel = prevVel * 0.8 + vel * 0.2;
			prevPv = Pv;

			if (SP != prevSP) { // generate new motion profile if target position has changed
				prevSP = SP;
				motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
						new MotionState(prevPv, prevVel,
								motionProfile.get(elapsedTime.seconds()).getA()),
						new MotionState(SP, 0, 0),
						MAX_V,
						MAX_A,
						MAX_J
				);
				elapsedTime = new ElapsedTime();
			}

			coefficients.kP = Kp;
			coefficients.kI = Ki;
			coefficients.kD = Kd;

			MotionState targetState = motionProfile.get(elapsedTime.seconds());
			double x = targetState.getX();
			double v = targetState.getV();
			double a = targetState.getA();
			controller.setTargetPosition(x);
			controller.setTargetVelocity(v);
			controller.setTargetAcceleration(a);

			power = 0//controller.update(Pv, prevVel)
					+ Kv * v + Ka * a + getKg(Pv);

			prevTime = time;
			telemetry.addData("targetPosition", String.valueOf(x));
			telemetry.addData("actualPosition", String.valueOf(Pv));
			telemetry.addData("targetVelocity", String.valueOf(v));
			telemetry.addData("actualVelocity", String.valueOf(prevVel));

			telemetry.update();
		}
		return power;
	}
}
