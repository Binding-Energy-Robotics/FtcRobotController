package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.CommandGroups.DropCone;
import org.firstinspires.ftc.teamcode.Framework.Commands.Claw.ToggleClaw;
import org.firstinspires.ftc.teamcode.Framework.Commands.Drive.MecDrive;
import org.firstinspires.ftc.teamcode.Framework.Commands.Flipper.FlipIn;
import org.firstinspires.ftc.teamcode.Framework.Commands.Flipper.FlipOut;
import org.firstinspires.ftc.teamcode.Framework.Commands.Rumble;
import org.firstinspires.ftc.teamcode.Framework.Commands.Slide.SetSlidePosition;
import org.firstinspires.ftc.teamcode.Framework.Commands.Slide.SetSlidePower;
import org.firstinspires.ftc.teamcode.Framework.Commands.TelemetryUpdate;
import org.firstinspires.ftc.teamcode.Framework.Commands.Wrist.MoveWrist;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Framework.subsystems.TeleDrive;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Wrist;

import java.util.Set;

@TeleOp
public class MainTeleop extends CommandOpMode {

    TeleDrive drive;
    Claw claw;
    Flipper flipper;
    LinearSlide slide;
    GamepadEx driver;
    GamepadEx gunner;

    Telemetry telemetry;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(super.telemetry,
                FtcDashboard.getInstance().getTelemetry());

        driver = new GamepadEx(gamepad1);
        gunner = new GamepadEx(gamepad2);
        Button Ad = new GamepadButton(gunner, GamepadKeys.Button.LEFT_BUMPER);
        Button dpadUp = new GamepadButton(gunner, GamepadKeys.Button.DPAD_UP);
        Button dpadDown = new GamepadButton(gunner, GamepadKeys.Button.DPAD_DOWN);
        Button dpadLeft = new GamepadButton(gunner, GamepadKeys.Button.DPAD_LEFT);
        Button dpadRight = new GamepadButton(gunner, GamepadKeys.Button.DPAD_RIGHT);
        Button Ag = new GamepadButton(gunner, GamepadKeys.Button.A);
        Button Bg = new GamepadButton(gunner, GamepadKeys.Button.B);
        Button Xg = new GamepadButton(gunner, GamepadKeys.Button.X);
        Button Yg = new GamepadButton(gunner, GamepadKeys.Button.Y);
        Button rightShoulder = new GamepadButton(gunner, GamepadKeys.Button.RIGHT_BUMPER);

        // Hardware initialization
        drive = new TeleDrive(hardwareMap);
        claw = new Claw(hardwareMap);
        slide = new LinearSlide(hardwareMap, telemetry, true);
        flipper = new Flipper(hardwareMap, telemetry);

        // Command setup
        DropCone dropCone = new DropCone(slide, flipper, claw);

        MecDrive mecDrive = new MecDrive(drive, () -> gamepad2.left_stick_y,
                () -> -gamepad2.left_stick_x, () -> -gamepad2.right_stick_x,
                () -> gunner.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1, telemetry);

        FlipOut flipOut = new FlipOut(flipper);
        FlipIn flipIn = new FlipIn(flipper);

        ParallelCommandGroup highJunction = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.HIGH),
                new FlipOut(flipper)
        );
        ParallelCommandGroup mediumJunction = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.MEDIUM),
                new FlipOut(flipper)
        );
        ParallelCommandGroup lowJunction = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.LOW),
                new FlipIn(flipper)
        );
        ParallelCommandGroup groundJunction = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.GROUND),
                new FlipIn(flipper)
        );
        ParallelCommandGroup coneHeight = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.ONE_CONE),
                new FlipIn(flipper)
        );
        ParallelCommandGroup twoConeHeight = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.TWO_CONE),
                new FlipIn(flipper)
        );
        ParallelCommandGroup threeConeHeight = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.THREE_CONE),
                new FlipIn(flipper)
        );
        ParallelCommandGroup fourConeHeight = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.FOUR_CONE),
                new FlipIn(flipper)
        );
        ParallelCommandGroup fiveConeHeight = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.FIVE_CONE),
                new FlipIn(flipper)
        );

        // Command Binding
        Ad.whenPressed(dropCone);
        Ad.whenPressed(() -> gamepad1.rumbleBlips(1));
        Ad.whenPressed(() -> gamepad2.rumbleBlips(1));

        dpadUp.whenPressed(highJunction);
        dpadRight.whenPressed(mediumJunction);
        dpadDown.whenPressed(lowJunction);
        dpadLeft.whenPressed(groundJunction);
        Ag.whenPressed(coneHeight);
        Bg.whenPressed(twoConeHeight);
        Xg.whenPressed(threeConeHeight);
        Yg.whenPressed(fourConeHeight);
        rightShoulder.whenPressed(fiveConeHeight);

        schedule(mecDrive, new TelemetryUpdate(telemetry));

        register(drive, claw, slide, flipper);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
