package org.firstinspires.ftc.teamcode.TeleOps;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

public class drive extends CommandOpMode {

    TeleDrive drive;
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


        MecDrive mecDrive = new MecDrive(hardwareMap, drive, () -> gamepad1.left_stick_y,
                () -> -gamepad1.left_stick_x, () -> -gamepad1.right_stick_x,
                () -> driver.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.1, telemetry);
        register(drive);

        telemetry.addData("Status", "Initialized");
        telemetry.update();



    }
}
