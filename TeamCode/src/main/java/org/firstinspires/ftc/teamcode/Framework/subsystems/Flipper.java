package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Flipper extends SubsystemBase {
	private HardwareMap hw;
	private ServoEx pitchServo;
	private ServoEx rollServo;
	private Telemetry t;

	public Flipper(HardwareMap hw, String pitchName, String rollName, Telemetry t) {
		this.hw = hw;
		this.pitchServo = new SimpleServo(hw, pitchName, 0, 180);
		this.rollServo = new SimpleServo(hw, rollName, 0, 180);
		this.t = t;
	}

	public Flipper(HardwareMap hw, Telemetry t) {
		this(hw, "pitch", "roll", t);
	}

	public void setPosition(double position) {
		pitchServo.setPosition(position);
		rollServo.setPosition(position);
	}

	@Override
	public void periodic() {
		t.addData("Pitch position", pitchServo.getPosition());
		t.addData("Roll position", pitchServo.getPosition());
	}
}
