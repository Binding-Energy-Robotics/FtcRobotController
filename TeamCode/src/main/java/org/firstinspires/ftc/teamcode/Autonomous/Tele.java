package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Framework.Commands.MecDrive;
import org.firstinspires.ftc.teamcode.Framework.subsystems.TeleDrive;

@TeleOp
public class Tele extends CommandOpMode {
    private GamepadEx gp1, gp2;
    private TeleDrive drive;

    @Override
    public void initialize() {

        // Subsystem Initialization

//        gp1 = new GamepadEx(gamepad1);
//        gp2 = new GamepadEx(gamepad2);
        drive = new TeleDrive(hardwareMap);
        register(drive);

        drive.setDefaultCommand(new MecDrive(drive,() -> -1 * gamepad1.touchpad_finger_1_x, () -> -1 * gamepad1.touchpad_finger_1_y, ()-> -1 * gamepad1.right_stick_x));
    }

}
