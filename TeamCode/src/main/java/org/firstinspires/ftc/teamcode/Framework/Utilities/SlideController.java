package org.firstinspires.ftc.teamcode.Framework.Utilities;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SlideController {
	public static double Kp = 0;
	public static double Ki = 0;
	public static double Kd = 0;

	public static double Kg = 0;
	public static double Kv = 0;
	public static double Ka = 0;

	private PIDController controller;

	public SlideController() {
		controller = new PIDController(Kp, Ki, Kd);
	}

	public double getPower(double Pv, double dt) {
		controller.setKp(Kp);
		controller.setKi(Ki);
		controller.setKd(Kd);
		return Kg + controller.getControl(Pv, dt);
	}
}
