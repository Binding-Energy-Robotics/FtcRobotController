package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wrist extends SubsystemBase {
	private HardwareMap hw;
	private ServoEx servo;
	private Telemetry t;

	public Wrist(HardwareMap hw, String name, Telemetry t) {
		this.hw = hw;
		this.servo = new SimpleServo(hw, name, 0, 180);
		servo.setPosition(0.3);
		this.t = t;
	}

	public void setPosition(double position) {
		servo.setPosition(position);
	}
}
