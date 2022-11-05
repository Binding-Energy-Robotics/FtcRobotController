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
import org.firstinspires.ftc.teamcode.Framework.Commands.Drive.MecDrive;
import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.OpenClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.Slide.SetSlideState;
import org.firstinspires.ftc.teamcode.Framework.Utilities.Direction;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideState;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Framework.subsystems.TeleDrive;

@TeleOp
public class MainTeleop extends CommandOpMode {

    TeleDrive drive;
    Claw claw;
    LinearSlide slide;
    GamepadEx driver;


    @Override
    public void initialize() {

        driver =  new GamepadEx(gamepad1);
        // Hardware initialization
        drive = new TeleDrive(hardwareMap);
//        claw = new Claw(hardwareMap, "claw");
        slide = new LinearSlide(hardwareMap, "slide");

        // Command setup
//        OpenClaw openClaw = new OpenClaw(claw);
//        CloseClaw closeClaw = new CloseClaw(claw);
        MecDrive mecDrive = new MecDrive(drive, () -> gamepad1.left_stick_x, () ->-1 * gamepad1.left_stick_y, () ->gamepad1.right_stick_x);
//        SetSlideState slideDown = new SetSlideState(slide, Direction.DOWN);
//        SetSlideState slideUp = new SetSlideState(slide, Direction.UP);

        // Button Setup
        Button dpadUp = new GamepadButton(driver, GamepadKeys.Button.DPAD_UP);
        Button dpadDown = new GamepadButton(driver, GamepadKeys.Button.DPAD_DOWN);
//        Button A = new GamepadButton(driver, GamepadKeys.Button.A);
//        Button B = new GamepadButton(driver, GamepadKeys.Button.B);
        TriggerReader rightTrigger = new TriggerReader(driver, GamepadKeys.Trigger.RIGHT_TRIGGER);
        TriggerReader leftTrigger = new TriggerReader(driver, GamepadKeys.Trigger.LEFT_TRIGGER);

        // Command Binding
//        dpadUp.whenPressed(slideUp);
//        dpadDown.whenPressed(slideDown);
//        A.whenPressed(openClaw);
//        B.whenPressed(closeClaw);
//        dpadUp.whileActiveContinuous(new InstantCommand(()->slide.setPower(1)));
//        dpadUp.whileActiveContinuous(new InstantCommand(()->slide.setPower(-1)));
        slide.setDefaultCommand(new InstantCommand(() -> {
            slide.setPower(driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        }));


        // Command scheduling and registration
//        schedule(openClaw, closeClaw, mecDrive);
        schedule(mecDrive);
//        register(drive, claw, slide);
        register(drive, slide);


    }
}
