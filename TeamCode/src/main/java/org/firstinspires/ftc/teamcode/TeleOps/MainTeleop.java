package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.Commands.DriverToggle;
import org.firstinspires.ftc.teamcode.Framework.Commands.AsyncDelay;
import org.firstinspires.ftc.teamcode.Framework.Commands.Drive.MecDrive;
import org.firstinspires.ftc.teamcode.Framework.Commands.Flipper.FlipIn;
import org.firstinspires.ftc.teamcode.Framework.Commands.Flipper.FlipOut;
import org.firstinspires.ftc.teamcode.Framework.Commands.Rumble;
import org.firstinspires.ftc.teamcode.Framework.Commands.Slide.SetSlidePosition;
import org.firstinspires.ftc.teamcode.Framework.Commands.Slide.SetSlidePower;
import org.firstinspires.ftc.teamcode.Framework.Commands.TelemetryUpdate;
import org.firstinspires.ftc.teamcode.Framework.Utilities.AutoEndPose;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;
import org.firstinspires.ftc.teamcode.Framework.subsystems.Flipper;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.Framework.subsystems.TeleDrive;

@TeleOp
public class MainTeleop extends CommandOpMode {

    TeleDrive drive;
    Claw claw;
    Flipper flipper;
    LinearSlide slide;
    GamepadEx driver;

    Telemetry telemetry;

    boolean isDriving;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        isDriving = true;

        telemetry = new MultipleTelemetry(super.telemetry,
                FtcDashboard.getInstance().getTelemetry());


        driver = new GamepadEx(gamepad1);
        Button rightTrigger = new Button() {
            @Override
            public boolean get() {
                return driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.1 && isDriving;
            }
        };
        Button dpadUp = new Button() {
            @Override
            public boolean get() {
                return driver.isDown(GamepadKeys.Button.DPAD_UP) && isDriving;
            }
        };
        Button dpadDown = new Button() {
            @Override
            public boolean get() {
                return driver.isDown(GamepadKeys.Button.DPAD_DOWN) && isDriving;
            }
        };
        Button dpadLeft = new Button() {
            @Override
            public boolean get() {
                return driver.isDown(GamepadKeys.Button.DPAD_LEFT) && isDriving;
            }
        };
        Button dpadRight = new Button() {
            @Override
            public boolean get() {
                return driver.isDown(GamepadKeys.Button.DPAD_RIGHT) && isDriving;
            }
        };
        Button A = new Button() {
            @Override
            public boolean get() {
                return driver.isDown(GamepadKeys.Button.A) && isDriving;
            }
        };
        Button B = new Button() {
            @Override
            public boolean get() {
                return driver.isDown(GamepadKeys.Button.B) && isDriving;
            }
        };
        Button X = new Button() {
            @Override
            public boolean get() {
                return driver.isDown(GamepadKeys.Button.X) && isDriving;
            }
        };
        Button Y = new Button() {
            @Override
            public boolean get() {
                return driver.isDown(GamepadKeys.Button.Y) && isDriving;
            }
        };
        Button rightShoulder = new Button() {
            @Override
            public boolean get() {
                return driver.isDown(GamepadKeys.Button.RIGHT_BUMPER) && isDriving;
            }
        };
        Button leftShoulder = new Button() {
            @Override
            public boolean get() {
                return driver.isDown(GamepadKeys.Button.LEFT_BUMPER);
            }
        };
        Button resetEncoder = new Button() {
            @Override
            public boolean get() {
                return driver.isDown(GamepadKeys.Button.A) && !isDriving;
            }
        };

        // Hardware initialization
        drive = new TeleDrive(hardwareMap);
        claw = new Claw(hardwareMap);
        slide = new LinearSlide(hardwareMap, telemetry, () -> isDriving);
        flipper = new Flipper(hardwareMap, telemetry, false);
        claw.open();

        // Command setup
        DriverToggle driverToggle = new DriverToggle(slide, flipper, claw);

        MecDrive mecDrive = new MecDrive(hardwareMap, drive, () -> gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x, () -> -gamepad1.right_stick_x,
                () -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1, telemetry);
        SetSlidePower manualMove = new SetSlidePower(slide,
                () -> (driver.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) -
                        driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)) * 0.5);


        ParallelCommandGroup highJunction = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.HIGH),
                new SequentialCommandGroup(
                        new AsyncDelay(0.1),
                        new FlipOut(flipper)
                )
        );
        ParallelCommandGroup mediumJunction = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.MEDIUM),
                new SequentialCommandGroup(
                        new AsyncDelay(0.1),
                        new FlipOut(flipper)
                )
        );
        ParallelCommandGroup lowJunction = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.LOW),
                new FlipIn(flipper)
        );
        ParallelCommandGroup groundJunction = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.GROUND),
                new SequentialCommandGroup(
                        new AsyncDelay(0.1),
                        new FlipIn(flipper)
                )
        );
        ParallelCommandGroup coneHeight = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.ONE_CONE),
                new SequentialCommandGroup(
                        new AsyncDelay(0.1),
                        new FlipIn(flipper)
                )
        );
        ParallelCommandGroup twoConeHeight = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.TWO_CONE),
                new SequentialCommandGroup(
                        new AsyncDelay(0.1),
                        new FlipIn(flipper)
                )
        );
        ParallelCommandGroup threeConeHeight = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.THREE_CONE),
                new SequentialCommandGroup(
                        new AsyncDelay(0.1),
                        new FlipIn(flipper)
                )
        );
        ParallelCommandGroup fourConeHeight = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.FOUR_CONE),
                new SequentialCommandGroup(
                        new AsyncDelay(0.1),
                        new FlipIn(flipper)
                )
        );
        ParallelCommandGroup fiveConeHeight = new ParallelCommandGroup(
                new SetSlidePosition(slide, LinearSlide.FIVE_CONE),
                new SequentialCommandGroup(
                        new AsyncDelay(0.1),
                        new FlipIn(flipper)
                )
        );

        SequentialCommandGroup rumbleTimes = new SequentialCommandGroup(
                new AsyncDelay(AutoEndPose::getTimer, AutoEndPose.ENDGAME_START - 5),
                new InstantCommand(AutoEndPose::clearTimer),

                new Rumble(gamepad1, 333),
                new AsyncDelay(1),
                new Rumble(gamepad1, 333),
                new AsyncDelay(1),
                new Rumble(gamepad1, 333),
                new AsyncDelay(1),
                new Rumble(gamepad1, 333),
                new AsyncDelay(1),
                new Rumble(gamepad1, 333),
                new AsyncDelay(26),

                new Rumble(gamepad1, 333),
                new AsyncDelay(1),
                new Rumble(gamepad1, 333),
                new AsyncDelay(1),
                new Rumble(gamepad1, 333),
                new AsyncDelay(1),
                new Rumble(gamepad1, 333),
                new AsyncDelay(1),
                new Rumble(gamepad1, 333)
        );

        // Command Binding
        rightTrigger.whenPressed(driverToggle);

        rightTrigger.whenPressed(() -> gamepad1.rumble(1, 1, 100));

        dpadUp.whenPressed(highJunction);
        dpadRight.whenPressed(mediumJunction);
        dpadDown.whenPressed(lowJunction);
        dpadLeft.whenPressed(groundJunction);
        A.whenPressed(coneHeight);
        B.whenPressed(twoConeHeight);
        X.whenPressed(threeConeHeight);
        Y.whenPressed(fourConeHeight);
        rightShoulder.whenPressed(fiveConeHeight);
        leftShoulder.whenPressed(() -> isDriving = !isDriving);
        resetEncoder.whenPressed(() -> slide.resetEncoder());

        schedule(mecDrive, manualMove, rumbleTimes, new TelemetryUpdate(telemetry));

        register(drive, claw, slide, flipper);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        flipper.setPosition(1);
    }
}
