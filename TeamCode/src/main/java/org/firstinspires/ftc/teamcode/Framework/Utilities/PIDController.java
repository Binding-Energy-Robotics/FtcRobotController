package org.firstinspires.ftc.teamcode.Framework.Utilities;

import com.qualcomm.robotcore.util.Range;

public class PIDController {
	private double Kp;
	private double Ki;
	private double Kd;
	private double I;
	private double Wi;

	private Double prevE;

	private double Sp;

	public PIDController(double Kp, double Ki, double Kd) {
		this.Kp = Kp;
		this.Ki = Ki;
		this.Kd = Kd;
		I = 0;
		Wi = 200;
		prevE = null;
		Sp = 0;
	}

	public double getControl(double Pv, double dt) { // basic PID control todo add cool stuff
		double e = Sp - Pv;
		double P = Kp * e;
		double D = 0;
		if (prevE != null) {
			if (Math.abs(e) <= Wi) {
				I += Ki * (e + prevE) / 2 * dt;
			}
			else {
				I = 0;
			}
			I = Range.clip(I, -0.2, 0.2);
			D = Kd * (e - prevE) / dt;
		}
		prevE = e;
		return P + I + D;
	}

	public void reset() {
		I = 0;
		prevE = null;
		Sp = 0;
	}

	public void setKp(double kp) { Kp = kp; }
	public void setKi(double ki) { Ki = ki; }
	public void setKd(double kd) { Kd = kd; }
	public void setSp(Double sp) { Sp = sp; }
	public void setWi(double wi) { Wi = wi; }
}
