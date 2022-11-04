package org.firstinspires.ftc.teamcode.Framework.subsystems;

import android.transition.Slide;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideConstants;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideState;

public class LinearSlide extends SubsystemBase {
    private HardwareMap hw;
    private String name;
    private MotorEx slideMotor;
    private SlideState state;
    private PIDFController controller;
    private SlideConstants slideConstants;

    public LinearSlide(final HardwareMap hw, final String name){
        this.hw = hw;
        this.name = name;
        this.state = SlideState.DOWN;
        this.slideConstants = new SlideConstants(this);
        this.controller = new PIDFController(slideConstants.KP, slideConstants.KI, slideConstants.KD, slideConstants.KF);

        slideMotor = new MotorEx(hw, "slideMotor");
    }

    public int getEncoderCount(){
        return slideMotor.getCurrentPosition();
    }

    public void setState(SlideState state){
        this.state = state;
    }
    public SlideState getState(){
        return this.state;
    }

    @Override
    public void periodic(){
        double output = controller.calculate(slideMotor.getCurrentPosition(), slideConstants.getTicks(state));
        slideMotor.setVelocity(output);
    }
}
