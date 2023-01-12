package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Flipper extends SubsystemBase {
	private HardwareMap hw;
	private ServoEx mainPitchServo;
	private ServoEx auxPitchServo;
//	private ServoEx rollServo;
	private Telemetry t;

	public Flipper(HardwareMap hw, String mainPitchName, String auxPitchName, String rollName, Telemetry t) {
		this.hw = hw;
		this.mainPitchServo = new SimpleServo(hw, mainPitchName, 0, 180);
		this.auxPitchServo = new SimpleServo(hw, auxPitchName, 0, 180);
		auxPitchServo.setInverted(true);
		this.t = t;
		setPosition(1);
	}

	public Flipper(HardwareMap hw, Telemetry t) {
		this(hw, "mainPitch", "auxPitch", "roll", t);
	}

	public void setPosition(double position) {
		mainPitchServo.setPosition(position);
		auxPitchServo.setPosition(position);
	}
}
