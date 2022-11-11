package org.firstinspires.ftc.teamcode.TeleOps;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.CloseClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.ToggleClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.Drive.MecDrive;
import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.OpenClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.Slide.SetSlidePower;
import org.firstinspires.ftc.teamcode.Framework.Commands.Slide.SetSlideState;
import org.firstinspires.ftc.teamcode.Framework.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Framework.Utilities.Direction;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideState;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Framework.subsystems.TeleDrive;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Wrist;

@TeleOp
public class MainTeleop extends CommandOpMode {

    TeleDrive drive;
    Claw claw;
    Wrist wrist;
    LinearSlide slide;
    GamepadEx driver;


    @Override
    public void initialize() {
        driver =  new GamepadEx(gamepad1);
        Button A = new GamepadButton(driver, GamepadKeys.Button.A);
        Button B = new GamepadButton(driver, GamepadKeys.Button.B);

        // Hardware initialization
        drive = new TeleDrive(hardwareMap);
        claw = new Claw(hardwareMap, "claw");
        slide = new LinearSlide(hardwareMap, "slide", telemetry);
        wrist = new Wrist(hardwareMap, "wrist");

        // Command setup
        ToggleClaw toggleClaw = new ToggleClaw(claw);

        MecDrive mecDrive = new MecDrive(drive, () -> gamepad1.left_stick_x,
                () -> -gamepad1.left_stick_y, () -> -gamepad1.right_stick_y,
                () -> driver.isDown(GamepadKeys.Button.LEFT_BUMPER));

        SetSlidePower slidePower = new SetSlidePower(slide,
                () -> driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) -
                        driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        MoveWrist moveWrist = new MoveWrist(wrist, () -> driver.getRightX());

        // Command Binding
        A.whenPressed(toggleClaw);

        schedule(mecDrive);
        schedule(slidePower);
        schedule(moveWrist);

        register(drive, claw, slide, wrist);
    }
}
