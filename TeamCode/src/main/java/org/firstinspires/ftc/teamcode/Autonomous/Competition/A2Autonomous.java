package org.firstinspires.ftc.teamcode.Autonomous.Competition;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.subsystems.AutoDrive;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Camera;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Wrist;

public class A2Autonomous extends CommandOpMode {
	AutoDrive drive;
	LinearSlide slide;
	Wrist wrist;
	Claw claw;
	Camera camera;

	int parkPosition = 0;

	@Override
	public void initialize() {
		Telemetry telemetry = new MultipleTelemetry(this.telemetry);

		drive = new AutoDrive(hardwareMap);
		slide = new LinearSlide(hardwareMap, telemetry);
		wrist = new Wrist(hardwareMap, telemetry);
		claw = new Claw(hardwareMap);
		camera = new Camera(hardwareMap);


		register(drive, slide, wrist, claw, camera);

		while (!isStarted()) {
			camera.periodic();
		}

		parkPosition = camera.getSide();
	}
}
