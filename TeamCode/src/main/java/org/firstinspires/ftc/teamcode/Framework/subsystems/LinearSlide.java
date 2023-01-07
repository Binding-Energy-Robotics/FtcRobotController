package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideController;

public class LinearSlide extends SubsystemBase {
    public static final int HIGH = 2200;
    public static final int MEDIUM = 1350;
    public static final int LOW = 500;
    public static final int BOTTOM = 0;
    public static final int FIVE_CONE = 430;
    public static final int FOUR_CONE = 330;
    public static final int THREE_CONE = 230;
    public static final int TWO_CONE = 80;
    public static final int ONE_CONE = 0;
    public static final int[] CONE_STACK = new int[] {
            0,
            ONE_CONE,
            TWO_CONE,
            THREE_CONE,
            FOUR_CONE,
            FIVE_CONE
    };

    private int stackSize;

    private HardwareMap hw;
    private String nameMain;
    private MotorEx slideMotor;
    private String nameAux;
    private MotorEx auxillaryMotor;
    private SlideController controller;
    private Telemetry t;

    private boolean usingPID;

    public LinearSlide(final HardwareMap hw,
                       String nameMain, String nameAux, Telemetry t, boolean usingPID){
        this.hw = hw;
        this.nameMain = nameMain;
        this.nameAux = nameAux;
        this.controller = new SlideController();
        slideMotor = new MotorEx(hw, nameMain);
        slideMotor.setInverted(true);
        slideMotor.encoder.setDirection(Motor.Direction.REVERSE);
        slideMotor.resetEncoder();
        auxillaryMotor = new MotorEx(hw, nameAux);
        this.t = t;
        this.usingPID = usingPID;
        stackSize = 5;
    }

    public LinearSlide(final HardwareMap hw, Telemetry t, boolean usingPID) {
        this(hw, "slideMain", "slideAux", t, usingPID);
    }

    public int getEncoderCount(){
        return slideMotor.getCurrentPosition();
    }

    public void setPower(double power){
        double position = slideMotor.getCurrentPosition();

        if (position < 10 && power < 0 || position > 2900 && power > 0) {
            power = 0;
        }

        power += controller.getKg(getEncoder()); // gravity feedforward

        slideMotor.set(power);
        auxillaryMotor.set(power);
    }

    public int getEncoder(){
        return slideMotor.getCurrentPosition();
    }

    public void setSlidePosition(int position) {
        controller.setTargetPosition(position);
    }

    public boolean isMovementFinished() {
        return controller.isMovementFinished();
    }

    public int popStackHeight() {
        int height = CONE_STACK[stackSize];
        if (stackSize > 0)
            stackSize--;
        return height;
    }

    @Override
    public void periodic() {
        t.addData("Height", slideMotor.getCurrentPosition());
        if (usingPID) {
            int position = slideMotor.getCurrentPosition();
            double power = controller.getPower(position);
            slideMotor.set(power);
            auxillaryMotor.set(power);
        }
    }
}
