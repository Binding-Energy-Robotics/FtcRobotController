package org.firstinspires.ftc.teamcode.Framework.Utilities.DriveSensors;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class UltrasonicGroup {
	private static final String LEFT_SIDE_NAME = "leftSideUltrasonic";
	private static final String LEFT_BACK_NAME = "leftBackUltrasonic";
	private static final String RIGHT_BACK_NAME = "rightBackUltrasonic";
	private static final String RIGHT_SIDE_NAME = "rightSideUltrasonic";

	private static final Pose2d LEFT_SIDE_POSE = new Pose2d(0, 0, Math.PI / 2);
	private static final Pose2d LEFT_BACK_POSE = new Pose2d(0, 0, Math.PI);
	private static final Pose2d RIGHT_BACK_POSE = new Pose2d(0, 0, Math.PI);
	private static final Pose2d RIGHT_SIDE_POSE = new Pose2d(0, 0, -Math.PI / 2);

	private static final String ULTRASONIC_TRIGGER_NAME = "ultrasonicTrigger";

	private UltrasonicSensor leftSide;
	private UltrasonicSensor leftBack;
	private UltrasonicSensor rightBack;
	private UltrasonicSensor rightSide;
	private UltrasonicSensor[] leadingGroup;
	private UltrasonicSensor[] laggingGroup;

	private DigitalChannel ultrasonicTrigger;
	private boolean triggerState;

	public UltrasonicGroup(HardwareMap hw) {
		leftSide = new UltrasonicSensor(hw, LEFT_SIDE_NAME, LEFT_SIDE_POSE);
		leftBack = new UltrasonicSensor(hw, LEFT_BACK_NAME, LEFT_BACK_POSE);
		rightBack = new UltrasonicSensor(hw, RIGHT_BACK_NAME, RIGHT_BACK_POSE);
		rightSide = new UltrasonicSensor(hw, RIGHT_SIDE_NAME, RIGHT_SIDE_POSE);
		leadingGroup = new UltrasonicSensor[] {
				leftSide,
				rightBack
		};
		laggingGroup = new UltrasonicSensor[] {
				leftBack,
				rightSide
		};

		triggerState = false;
		ultrasonicTrigger = hw.get(DigitalChannel.class, ULTRASONIC_TRIGGER_NAME);
		ultrasonicTrigger.setMode(DigitalChannel.Mode.OUTPUT);

		// send initial pulse to read sensors
		ultrasonicTrigger.setState(false);
		ultrasonicTrigger.setState(true);
		ultrasonicTrigger.setState(false);
	}

	public void trigger() {
		triggerState = !triggerState;
		ultrasonicTrigger.setState(triggerState);
	}

	public void read() {
		if (triggerState) {
			for (int i = 0; i < laggingGroup.length; i++) {
				laggingGroup[i].read();
			}
		}
		else {
			for (int i = 0; i < leadingGroup.length; i++) {
				leadingGroup[i].read();
			}
		}
	}
}
