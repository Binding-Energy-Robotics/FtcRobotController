package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Framework.subsystems.TeleDrive;

public class Tele extends CommandOpMode {
    private GamepadEx gp1, gp2;
    private TeleDrive drive;

    @Override
    public void initialize() {

        // Subsystem Initialization
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        drive = new TeleDrive(hardwareMap);
        register(drive);






    }
}
