package org.firstinspires.ftc.teamcode.Framework.Utilities;

import com.qualcomm.robotcore.util.Range;

public class PIDController {
	private double Kp;
	private double Ki;
	private double Kd;
	private double I;

	private Double prevE;

	private Double Sp;

	public PIDController(double Kp, double Ki, double Kd) {
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		I = 0;
		prevE = null;
		Sp = null;
	}

	public double getControl(double Pv, double dt) { // basic PID control todo add cool stuff
		double e = Sp - Pv;
		double P = Kp * e;
		double D = 0;
		if (prevE != null) {
			I += Ki * (e + prevE) / 2 * dt;
			I = Range.clip(I, -0.8, 0.8);
			D = Kd * (e - prevE) / dt;
		}
		prevE = e;
		return P + I + D;
	}

	public void reset() {
		I = 0;
		prevE = null;
		Sp = null;
	}

	public void setKp(double kp) { Kp = kp; }
	public void setKi(double ki) { Ki = ki; }
	public void setKd(double kd) { Kd = kd; }
	public void setSp(Double sp) { Sp = sp; }
}
