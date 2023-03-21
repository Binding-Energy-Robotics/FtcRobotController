package org.firstinspires.ftc.teamcode.Framework.Utilities;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.PIDEx;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficientsEx;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.filter.DefaultMeasurementModel;
import org.apache.commons.math3.filter.DefaultProcessModel;
import org.apache.commons.math3.filter.MeasurementModel;
import org.apache.commons.math3.filter.ProcessModel;
import org.apache.commons.math3.linear.Array2DRowRealMatrix;
import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.apache.commons.math3.filter.KalmanFilter;

// good shit, see Ben Caunt
// Kd = 2 * sqrt(Ka * Kp) - Kv, Kp >= Kv * Kv / (4 * Ka)
@Config
public class SlideController {
	public static double Kg = 0.1; // tuned by Alex Prichard on 12 Jan 2023

	public static double Kv = 0.00022; // tuned by Alex Prichard on 12 Jan 2023
	public static double Ka = 0.00001; // tuned by Alex Prichard on 12 Jan 2023
	public static double Ks = 0.08;

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

	private KalmanFilter kalmanFilter;
	private ProcessModel model;
	private MeasurementModel measurementModel;

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

		Array2DRowRealMatrix A = new Array2DRowRealMatrix(new double[][] {
				{ 0, 1, 0 },
				{ 0, -Kv / Ka, 1 / Ka },
				{ 0, 0, 0 }
		});
		Array2DRowRealMatrix B = new Array2DRowRealMatrix(new double[][] {
				{ 0 },
				{ 1 / Ka },
				{ 0 }
		});
		Array2DRowRealMatrix Q = new Array2DRowRealMatrix(new double[][] {
				{ 10, 0, 0 },
				{ 0, 20, 0 },
				{ 0, 0, 0.3 }
		});
		Array2DRowRealMatrix initError = new Array2DRowRealMatrix(new double[][] {
				{ 1, 0, 0 },
				{ 0, 1, 0 },
				{ 0, 0, 0.01 }
		});
		ArrayRealVector initState = new ArrayRealVector(new double[] { 0, 0, 0 });

		Array2DRowRealMatrix C = new Array2DRowRealMatrix(new double[][] {
				{ 1, 0, 0 },
				{ 0, 1, 0 }
		});
		Array2DRowRealMatrix R = new Array2DRowRealMatrix(new double[][] {
				{ 5, 0 },
				{ 0, 5 }
		});

		double loopTime = 0.03;

		RealMatrix taylor = taylorSeries(A, loopTime, 10);
		RealMatrix Bd = taylor.multiply(B);
		RealMatrix Ad = A.multiply(taylor).add(MatrixUtils.createRealIdentityMatrix(3));

		model = new DefaultProcessModel(Ad, Bd, Q, initState, initError);
		measurementModel = new DefaultMeasurementModel(C, R);
		kalmanFilter = new KalmanFilter(model, measurementModel);
	}

	private String printMat(RealMatrix mat) {
		StringBuilder toPrint = new StringBuilder("\n{\n");
		for (int i = 0; i < mat.getRowDimension(); i++) {
			for (int j = 0; j < mat.getColumnDimension(); j++) {
				if (j == 0) {
					toPrint.append("\t{").append(mat.getEntry(i, j));
				}
				else {
					toPrint.append(", ").append(mat.getEntry(i, j));
				}
			}
			toPrint.append("}\n");
		}
		toPrint.append("}");
		return toPrint.toString();
	}

	private RealMatrix taylorSeries(RealMatrix A, double dt, int terms) {
		RealMatrix exponentialA = MatrixUtils.createRealIdentityMatrix(3);
		RealMatrix acc = MatrixUtils.createRealIdentityMatrix(3).scalarMultiply(dt);
		double k = 1;
		double T = dt;
		for (int i = 2; i <= terms; i++) {
			exponentialA = exponentialA.multiply(A);
			T *= dt;
			k /= i;
			RealMatrix term = exponentialA.scalarMultiply(T * k);
			acc = acc.add(term);
		}
		return acc;
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

	private RealMatrix symmetrize(RealMatrix P) {
		for (int i = 0; i < 2; i++) {
			for (int j = i + 1; j < 3; j++) {
				double first = P.getEntry(i, j);
				double second = P.getEntry(j, i);
				double avg = (first + second) * 0.5;
				P.setEntry(i, j, avg);
				P.setEntry(j, i, avg);
			}
		}
		return P;
	}

	public double getPower(int Pv) {
		LogData.addData("Slide power", power);
		LogData.addData("Slide encoder", Pv);

		long time = System.nanoTime();
		double dt = (time - prevTime) * 1.0e-9;

		prevTime = time;

		double velMeasurement = (Pv - prevPos) / dt;
		prevPos = Pv;

		kalmanFilter.predict(new double[] { u });
		RealMatrix P = model.getProcessNoise();
		RealMatrix A = model.getStateTransitionMatrix();
		RealMatrix B = model.getControlMatrix();
		RealMatrix Q = model.getProcessNoise();
		RealVector stateVector = kalmanFilter.getStateEstimationVector();
		model = new DefaultProcessModel(
				A,
				B,
				Q,
				stateVector,
				A.multiply(P).multiply(A.transpose()).add(Q)
		);
		kalmanFilter = new KalmanFilter(model, measurementModel);
		kalmanFilter.correct(new double[] { Pv, velMeasurement });
		double[] stateEstimate = kalmanFilter.getStateEstimation();

		prevPv = stateEstimate[0];
		prevVel = stateEstimate[1];
		double adrCompensation = stateEstimate[2] * 0.2;

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

		if (!isMovementFinished) {
			u = controller.calculate(x, Pv) + v * Kv + a * Ka - adrCompensation;
			power = u + Kg + Math.copySign(Ks, v);
		}
		else {
			u = controller.calculate(x, Pv) - adrCompensation;
			power = u + Kg;
		}

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
