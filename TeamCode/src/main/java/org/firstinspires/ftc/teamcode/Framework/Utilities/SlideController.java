package org.firstinspires.ftc.teamcode.Framework.Utilities;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.Util;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SlideController {
	public static double KP = 0.005;
	public static double KI = 0.00001;
	public static double KD = 0;

	public static double KG = 0.1;
	public static double KV = 0;
	public static double KA = 0;

	public static double SP = 0;

	private PIDController controller;

	private long prevTime;
	private FtcDashboard dash;
	private Telemetry telemetry;

	public SlideController() {
		controller = new PIDController(KP, KI, KD);
		controller.setSp(SP);
		prevTime = System.nanoTime();
		dash = FtcDashboard.getInstance();
		telemetry = dash.getTelemetry();
	}

	public double getPower(double Pv, double dt) {
		SP = Range.clip(SP, 0, 1400);
		controller.setKp(KP);
		controller.setKi(KI);
		controller.setKd(KD);
		controller.setSp(SP);
		double power = KG + controller.getControl(Pv, dt);
		long time = System.nanoTime();
		if (time - prevTime > 1e7) {
			prevTime = time;
			telemetry.addData("Sp", String.valueOf(SP));
			telemetry.addData("Pv", String.valueOf(Pv));
			telemetry.addData("E", String.valueOf(SP - Pv));
			telemetry.addData("Power",
					String.valueOf(1000 * Range.clip(power, -1, 1)));
			telemetry.update();
		}
		return power;
	}
}
