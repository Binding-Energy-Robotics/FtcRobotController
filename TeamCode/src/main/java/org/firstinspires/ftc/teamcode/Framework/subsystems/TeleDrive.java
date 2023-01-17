package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class TeleDrive extends SubsystemBase {

    private MotorEx frontR, frontL, backL, backR;
    private MecanumDrive drive;
    private ServoEx odometryServo;
    private BNO055IMU imu;

    public TeleDrive(HardwareMap hw){
        this.frontR = new MotorEx(hw, "rightFront");
        this.frontL = new MotorEx(hw, "leftFront");
        this.backR = new MotorEx(hw, "rightRear");
        this.backL = new MotorEx(hw, "leftRear");
        this.drive = new MecanumDrive(frontL,frontR, backL, backR);
        odometryServo = new SimpleServo(hw, "odometryServo", 0, 180);
        odometryServo.setPosition(1);
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turn, double heading){
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turn, heading);
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turn){
        drive.driveRobotCentric(strafeSpeed, forwardSpeed, turn);
    }

    public void driveWithMultiplier(double strafeSpeed, double forwardSpeed,
                                    double turn, double heading, double multiplier){
        drive.driveFieldCentric(strafeSpeed * multiplier,
                forwardSpeed * multiplier, turn * multiplier, heading);
    }
}
