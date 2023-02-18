package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Framework.Commands.AsyncDelay;
import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;

@Disabled
@Autonomous
public class Test extends CommandOpMode {
	@Override
	public void initialize() {
		Claw claw = new Claw(hardwareMap);
		claw.open();

		SequentialCommandGroup run = new SequentialCommandGroup(
				new AsyncDelay(1),
				new AsyncDelay(10),
				new CloseClaw(claw)
		);

		register(claw);

		schedule(run);
	}
}
