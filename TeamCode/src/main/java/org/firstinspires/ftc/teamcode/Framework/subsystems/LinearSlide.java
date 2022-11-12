package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.Utilities.PIDController;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideConstants;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideController;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideState;

public class LinearSlide extends SubsystemBase {
    private HardwareMap hw;
    private String name;
    private MotorEx slideMotor;
    private SlideState state;
    private SlideController controller;
    private SlideConstants slideConstants;
    private Telemetry t;

    private boolean usingPID;
    private long prevTime;

    public LinearSlide(final HardwareMap hw, final String name, Telemetry t){
        this.hw = hw;
        this.name = name;
        this.state = SlideState.DOWN;
//        this.slideConstants = new SlideConstants(this);
        this.controller = new SlideController();
        slideMotor = new MotorEx(hw, name);
        slideMotor.resetEncoder();
        this.t = t;
        usingPID = true;
        this.prevTime = System.nanoTime();
    }

    public int getEncoderCount(){
        return slideMotor.getCurrentPosition();
    }

    public void setState(SlideState state){
        this.state = state;
    }

    public void setPower(double power){
        double position = slideMotor.getCurrentPosition();

        if (position < 0 && power < 0 || position > 1750 && power > 0) {
            power = 0;
        }

        slideMotor.set(power);
    }
    public SlideState getState(){
        return this.state;
    }

    public int getEncoder(){
        return slideMotor.getCurrentPosition();
    }

    @Override
    public void periodic() {
        long time = System.nanoTime();
        double dt = (time - prevTime) * 1.0e-9;
        if (usingPID) {
            int position = slideMotor.getCurrentPosition();
            double power = controller.getPower(position, dt);
            slideMotor.set(power);
        }
    }
}
