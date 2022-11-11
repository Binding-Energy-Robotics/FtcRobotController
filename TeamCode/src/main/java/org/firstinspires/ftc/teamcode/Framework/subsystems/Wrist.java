package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Wrist extends SubsystemBase {
	private HardwareMap hw;
	private ServoEx servo;

	public Wrist(HardwareMap hw, String name) {
		this.hw = hw;
		this.servo = new SimpleServo(hw, name, 0, 180);
		servo.setPosition(0.5);
	}

	public void setPosition(double position) {
		servo.setPosition(position);
	}
}
