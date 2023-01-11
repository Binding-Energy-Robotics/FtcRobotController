package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.ToggleClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.Drive.MecDrive;
import org.firstinspires.ftc.teamcode.Framework.Commands.Flipper.FlipIn;
import org.firstinspires.ftc.teamcode.Framework.Commands.Flipper.FlipOut;
import org.firstinspires.ftc.teamcode.Framework.Commands.Rumble;
import org.firstinspires.ftc.teamcode.Framework.Commands.Slide.SetSlidePower;
import org.firstinspires.ftc.teamcode.Framework.Commands.TelemetryUpdate;
import org.firstinspires.ftc.teamcode.Framework.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Framework.subsystems.TeleDrive;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Wrist;

@TeleOp
public class MainTeleop extends CommandOpMode {

    TeleDrive drive;
    Claw claw;
    Flipper flipper;
    LinearSlide slide;
    GamepadEx driver;
    GamepadEx helper;

    Telemetry telemetry;

    @Override
    public void initialize() {
        telemetry = new MultipleTelemetry(super.telemetry,
                FtcDashboard.getInstance().getTelemetry());

        driver = new GamepadEx(gamepad1);
        helper = new GamepadEx(gamepad2);
        Button A = new GamepadButton(driver, GamepadKeys.Button.A);
        Button dpadUp = new GamepadButton(helper, GamepadKeys.Button.DPAD_UP);
        Button dpadDown = new GamepadButton(helper, GamepadKeys.Button.DPAD_DOWN);

        // Hardware initialization
        drive = new TeleDrive(hardwareMap);
        claw = new Claw(hardwareMap, "claw");
        slide = new LinearSlide(hardwareMap, telemetry, false);
        flipper = new Flipper(hardwareMap, telemetry);

        // Command setup
        ToggleClaw toggleClaw = new ToggleClaw(claw);

        MecDrive mecDrive = new MecDrive(drive, () -> gamepad1.left_stick_y,
                () -> gamepad1.left_stick_x, () -> gamepad1.right_stick_x,
                () -> driver.isDown(GamepadKeys.Button.LEFT_BUMPER), telemetry);

        SetSlidePower slidePower = new SetSlidePower(slide,
                () -> helper.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) -
                        helper.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

        FlipOut flipOut = new FlipOut(flipper);
        FlipIn flipIn = new FlipIn(flipper);

        // Command Binding
        A.whenPressed(toggleClaw);
        A.whenPressed(() -> gamepad1.rumbleBlips(1));
        A.whenPressed(() -> gamepad2.rumbleBlips(1));

        dpadUp.whenPressed(flipOut);
        dpadDown.whenPressed(flipIn);

        schedule(mecDrive, slidePower, new TelemetryUpdate(telemetry));

        register(drive, claw, slide, flipper);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
