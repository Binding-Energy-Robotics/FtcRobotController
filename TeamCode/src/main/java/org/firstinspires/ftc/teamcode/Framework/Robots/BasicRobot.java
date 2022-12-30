package org.firstinspires.ftc.teamcode.Framework.Robots;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Drivetrain;

public class BasicRobot extends Robot {

    private Drivetrain drivetrain;

    public BasicRobot(HardwareMap hw){
        drivetrain = new Drivetrain(hw);
    }


}
