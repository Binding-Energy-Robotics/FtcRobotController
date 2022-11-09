package org.firstinspires.ftc.teamcode.Framework.subsystems;

import android.transition.Slide;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideConstants;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideState;

public class LinearSlide extends SubsystemBase {
    private HardwareMap hw;
    private String name;
    private MotorEx slideMotor;
    private SlideState state;
    private PIDFController controller;
    private SlideConstants slideConstants;
    private static int STOP_VALUE;
    private Telemetry t;

    public LinearSlide(final HardwareMap hw, final String name, Telemetry t){
        this.hw = hw;
        this.name = name;
        this.state = SlideState.DOWN;
//        this.slideConstants = new SlideConstants(this);
        this.controller = new PIDFController(SlideConstants.KP, SlideConstants.KI, SlideConstants.KD, SlideConstants.KF);
        slideMotor = new MotorEx(hw, "slideMotor");
        STOP_VALUE = slideMotor.getCurrentPosition();
        this.t = t;
    }

    public int getEncoderCount(){
        return slideMotor.getCurrentPosition();
    }

    public void setState(SlideState state){
        this.state = state;
    }

    public void setPower(double power){
        t.addData("Current Position", slideMotor.getCurrentPosition());
        t.addData("Stop Position", STOP_VALUE);
        t.addData("Power", power);
        t.update();
//        if(slideMotor.getCurrentPosition() < STOP_VALUE && power < 0){
//            slideMotor.set(0);
//        }
        slideMotor.set(power);
    }
    public SlideState getState(){
        return this.state;
    }

    public int getEncoder(){
        return slideMotor.getCurrentPosition();
    }

    public int getStopValue(){
        return STOP_VALUE;
    }

//
//    @Override
//    public void periodic(){
//        double output = controller.calculate(slideMotor.getCurrentPosition(), slideConstants.getTicks(state));
//        slideMotor.setVelocity(output);
//    }

    @Override
    public void periodic(){
        t.addData("Current Position", slideMotor.getCurrentPosition());
        t.addData("Stop Position", STOP_VALUE);
        t.update();
    }
}
