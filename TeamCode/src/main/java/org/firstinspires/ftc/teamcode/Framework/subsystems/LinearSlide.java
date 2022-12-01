package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideConstants;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideController;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideState;

public class LinearSlide extends SubsystemBase {
    private HardwareMap hw;
    private String nameMain;
    private MotorEx slideMotor;
    private String nameAux;
    private MotorEx auxillaryMotor;
    private SlideState state;
    private SlideController controller;
    private SlideConstants slideConstants;
    private Telemetry t;

    private boolean usingPID;

    public LinearSlide(final HardwareMap hw, String nameMain, String nameAux, Telemetry t){
        this.hw = hw;
        this.nameMain = nameMain;
        this.nameAux = nameAux;
        this.state = SlideState.DOWN;
//        this.slideConstants = new SlideConstants(this);
        this.controller = new SlideController();
        slideMotor = new MotorEx(hw, nameMain);
        slideMotor.setInverted(true);
        slideMotor.resetEncoder();
        auxillaryMotor = new MotorEx(hw, nameAux);
        this.t = t;
        usingPID = false;
    }

    public int getEncoderCount(){
        return slideMotor.getCurrentPosition();
    }

    public void setState(SlideState state){
        this.state = state;
    }

    public void setPower(double power){
        double position = slideMotor.getCurrentPosition();

        t.addData("Position", position);
        t.update();
//        if (position < 0 && power < 0 || position > 1750 && power > 0) {
//            power = 0;
//        }

        slideMotor.set(power);
        auxillaryMotor.set(power);
    }
    public SlideState getState(){
        return this.state;
    }

    public int getEncoder(){
        return slideMotor.getCurrentPosition();
    }

    @Override
    public void periodic() {
        if (usingPID) {
            int position = slideMotor.getCurrentPosition();
            double power = controller.getPower(position);
            slideMotor.set(power);
        }
    }
}
