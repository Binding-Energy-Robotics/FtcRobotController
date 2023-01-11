package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Claw extends SubsystemBase {
    private HardwareMap hw;
    private ServoEx clawServo;
    private boolean servoClosed;

    public Claw(HardwareMap hw, String name){
        this.hw = hw;
        clawServo = new SimpleServo(hw, name, 0, 180);
        close();
    }

    public Claw(HardwareMap hw) {
        this(hw, "claw");
    }

    public void open(){
        clawServo.setPosition(0);
        servoClosed = false;
    }

    public void close(){
        clawServo.setPosition(1);
        servoClosed = true;
    }

    public void toggle() {
        if (servoClosed) {
            open();
        } else {
            close();
        }
    }
}
