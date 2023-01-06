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
    private int toggleCount;
    private Telemetry telemetry;

    public Claw(HardwareMap hw, String name, Telemetry t){
        this.hw = hw;
        clawServo = new SimpleServo(hw, name, 0, 180);
        telemetry = t;
        open();
    }

    public void open(){
        clawServo.setPosition(0.5);
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
