package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Framework.subsystems.TeleDrive;

@TeleOp
public class TouchpadDrive extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        TeleDrive td = new TeleDrive(hardwareMap);


        waitForStart();
        while(opModeIsActive() && !isStopRequested()){

                td.drive(-1 * gamepad1.touchpad_finger_1_x, -1 * gamepad1.touchpad_finger_1_y, -1 * gamepad1.right_stick_x);
                telemetry.addData("X: ", gamepad1.touchpad_finger_1_x);
                telemetry.addData("Y: ", gamepad1.touchpad_finger_1_y);


            telemetry.update();
        }
    }
}
