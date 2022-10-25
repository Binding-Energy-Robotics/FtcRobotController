package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TeleDrive extends SubsystemBase {

    private MotorEx frontR, frontL, backL, backR;
    private MecanumDrive drive;

    public TeleDrive(HardwareMap hw){
        this.frontR = new MotorEx(hw, "frontR");
        this.frontL = new MotorEx(hw, "frontL");
        this.backR = new MotorEx(hw, "backR");
        this.backL = new MotorEx(hw, "backL");
        this.drive = new MecanumDrive(frontL,frontR, backL, backR);
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turn, double heading){
        drive.driveFieldCentric(strafeSpeed, forwardSpeed, turn, heading);
    }








}
