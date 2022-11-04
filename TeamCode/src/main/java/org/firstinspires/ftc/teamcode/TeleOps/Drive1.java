package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Framework.Commands.Drive.TankDriveCommand;
import org.firstinspires.ftc.teamcode.Framework.subsystems.TankDriveTrain;

public class Drive1 extends CommandOpMode {

    public CommandScheduler scheduler;
    private GamepadEx gp1;

    @Override
    public void initialize() {
        scheduler = CommandScheduler.getInstance();
        gp1 = new GamepadEx(gamepad1);

        TankDriveTrain driveTrain = new TankDriveTrain(hardwareMap);
        TankDriveCommand driveCommand = new TankDriveCommand(driveTrain, ()->gp1.getLeftY(), ()-> gp1.getRightY());

        register(driveTrain);
        driveTrain.setDefaultCommand(driveCommand);
    }
}
