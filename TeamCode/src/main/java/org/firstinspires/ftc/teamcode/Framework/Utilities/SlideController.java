package org.firstinspires.ftc.teamcode.Framework.Utilities;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// good shit, see Ben Caunt
// Kd = 2 * sqrt(Ka * Kp) - Kv, Kp >= Kv * Kv / (4 * Ka)
@Config
public class SlideController {
	public static double Kg = 0.1; // tuned by Alex Prichard on 12 Jan 2023

	public static double Kv = 0.00022; // tuned by Alex Prichard on 12 Jan 2023
	public static double Ka = 0.00001; // tuned by Alex Prichard on 12 Jan 2023
	public static double Ks = 0.08;

	private static final RealMatrix A = new Array2DRowRealMatrix(new double[][] {
			{ 0, 0 },
			{ 0, 0 }
	});
	private static final RealMatrix B = new Array2DRowRealMatrix(new double[][] {
			{ 0 },
			{ 0 }
	});
	private static final RealMatrix Q = new Array2DRowRealMatrix(new double[][] {
			{ 0, 0 },
			{ 0, 0 }
	});
	private static final RealMatrix R = new Array2DRowRealMatrix(new double[][] {
			{ 0, 0 },
			{ 0, 0 }
	});

	public static double MAX_V = 1_000; // tuned by Alex Prichard on 20 Jan 2023
	public static double MAX_A = 10_000; // tuned by Alex Prichard on 20 Jan 2023
	public static double MAX_J = 30_000; // tuned by Alex Prichard on 20 Jan 2023

	public static double Kp = 5 * Kv * Kv / (4 * Ka); // tuned by Alex Prichard on 20 Jan 2023
	public static double Ki = 0;//0.0000001; // tuned by Alex Prichard on 13 Jan 2023
	public static double Kd = 2 * 2 * Math.sqrt(Kp * Ka) - Kv; // see Ben Caunt's paper for more details
	public static double Imax = 0;//0.2 / Ki;
	public static double stabilityThreshold = 25;
	private PIDCoefficientsEx coefficients;

	public double prevSP = 0;
	public static double SP = 0;

	private double prevPv = 0;
	private double prevVel = 0;

	private double power = 0;

	private PIDEx controller;

	private SlideFilter kalmanFilter;

	private MotionProfile motionProfile;
	private ElapsedTime elapsedTime;

	private long prevTime;
	private FtcDashboard dash;
	private Telemetry telemetry;
	private boolean isMovementFinished = false;

	private double u = 0;
	private int prevPos = 0;

	public SlideController() {
		coefficients = new PIDCoefficientsEx(Kp, Ki, Kd, Imax, stabilityThreshold, 0.8);
		controller = new PIDEx(coefficients);
		prevTime = System.nanoTime();
		dash = FtcDashboard.getInstance();
		telemetry = dash.getTelemetry();
		motionProfile = staticProfile();
		elapsedTime = new ElapsedTime();
		kalmanFilter = new SlideFilter(A, B, Q, R);
	}

	private MotionProfile staticProfile() {
		return MotionProfileGenerator.generateSimpleMotionProfile(
				new MotionState(prevSP, 0 , 0),
				new MotionState(prevSP, 0 , 0),
				MAX_V,
				MAX_A,
				MAX_J
		);
	}

	public double getKg() {
		return Kg;
	}

	public void setTargetPosition(int Sp) {
		SP = Sp;
	}

	public double getPower(int Pv) {
		LogData.addData("Slide power", power);
		LogData.addData("Slide encoder", Pv);

		long time = System.nanoTime();
		double dt = (time - prevTime) * 1.0e-9;

		prevTime = time;

		double velMeasurement = (Pv - prevPos) / dt;
		prevPos = Pv;

		kalmanFilter.predict(u);
		kalmanFilter.correct(Pv, velMeasurement);

		prevPv = kalmanFilter.getPosition();
		prevVel = kalmanFilter.getVelocity();

		if (SP != prevSP) { // generate new motion profile if target position has changed
			try {
				motionProfile = MotionProfileGenerator.generateSimpleMotionProfile(
						new MotionState(prevPv, prevVel,
								motionProfile.get(elapsedTime.seconds()).getA()),
						new MotionState(SP, 0, 0),
						MAX_V,
						MAX_A,
						MAX_J
				);
				prevSP = SP;
				isMovementFinished = false;
				elapsedTime = new ElapsedTime();
			}
			catch (NullPointerException e) {
				e.printStackTrace();
			}
		}

		if (motionProfile.start().getX() < motionProfile.end().getX()) { // profile goes up
			if (prevPv > motionProfile.end().getX() - 10) {
				motionProfile = staticProfile();
				isMovementFinished = true;
			}
		}
		else if (prevPv < motionProfile.end().getX() + 10) {
			motionProfile = staticProfile();
			isMovementFinished = true;
		}

		coefficients.Kp = Kp;
		coefficients.Ki = Ki;
		coefficients.Kd = Kd;

		MotionState targetState = motionProfile.get(elapsedTime.seconds());
		double x = targetState.getX();
		double v = targetState.getV();
		double a = targetState.getA();

		u = controller.calculate(x, Pv);
		if (!isMovementFinished) {
			u += v * Kv + a * Ka;
		}
		power = u + Kg + Math.copySign(Ks, prevVel);

		telemetry.addData("position", Pv);
		telemetry.addData("target position", SP);
		telemetry.addData("power", 500 * power);

		if (Pv < 10 && power < 0 || Pv > 600 && power > 0) {
			return 0;
		}

		return power;
	}

	public boolean isMovementFinished() {
		return isMovementFinished;
	}

	public void timeOut() {
		if (isMovementFinished) return;
		motionProfile = staticProfile();
		isMovementFinished = true;
	}
}
