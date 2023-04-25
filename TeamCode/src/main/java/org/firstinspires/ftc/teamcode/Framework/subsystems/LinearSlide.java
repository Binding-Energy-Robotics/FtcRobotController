package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideController;

import java.util.function.BooleanSupplier;

public class LinearSlide extends SubsystemBase {
    public static final int HIGH = 612;
    public static final int MEDIUM = 320;
    public static final int LOW = 520;
    public static final int GROUND = 20;

    public static final int sus_offset = 40;
    public static final int FIVE_CONE = 175;
    public static final int FOUR_CONE = 150;
    public static final int THREE_CONE = 110;
    public static final int TWO_CONE = 40;
    public static final int ONE_CONE = sus_offset;

    public static final int[] CONE_STACK = new int[] {
            0,
            0,
            40 + sus_offset,
            100 + sus_offset,
            140 + sus_offset,
            170 + sus_offset
    };

    public static final int[] TELEOP_HEIGHTS = new int[] {
            ONE_CONE,
            TWO_CONE,
            THREE_CONE,
            FOUR_CONE,
            FIVE_CONE
    };

    private int targetHeight;

    private int stackSize;

    private HardwareMap hw;
    private MotorEx[] slideMotors = new MotorEx[2];
    private SlideController controller;
    private Telemetry t;

    private BooleanSupplier usingPID;

    public LinearSlide(final HardwareMap hw, String[] motorNames,
                       Telemetry t, BooleanSupplier usingPID) {
        this.hw = hw;
        this.controller = new SlideController();
        for (int i = 0; i < 2; i++) {
            slideMotors[i] = new MotorEx(hw, motorNames[i]);
        }
        slideMotors[0].setInverted(true);
        slideMotors[0].resetEncoder();
        this.t = t;
        this.usingPID = usingPID;
        stackSize = 5;
    }

    public LinearSlide(final HardwareMap hw, Telemetry t, BooleanSupplier usingPID) {
        this(hw, new String[]{
                "rightSpool", "leftSpool"
        }, t, usingPID);
    }

    public LinearSlide(final HardwareMap hw, String[] motorNames, Telemetry t, boolean usingPID) {
        this(hw, motorNames, t, () -> usingPID);
    }

    public LinearSlide(final HardwareMap hw, Telemetry t, boolean usingPID) {
        this(hw, t, () -> usingPID);
    }

    public LinearSlide(final HardwareMap hw, Telemetry t) {
        this(hw, t, true);
    }

    public void setPower(double power){
        if (usingPID.getAsBoolean())
            return;

        power += controller.getKg(); // gravity feedforward

        for (int i = 0; i < 2; i++) {
            slideMotors[i].set(power);
        }
    }

    public int getEncoder(){
        return slideMotors[0].getCurrentPosition();
    }

    public void setSlidePosition(int position) {
        targetHeight = position;
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

    public boolean isDown() {
        for (int coneHeight: CONE_STACK) {
            if (controller.prevSP == coneHeight)
                return true;
        }
        for (int coneHeight: TELEOP_HEIGHTS) {
            if (controller.prevSP == coneHeight)
                return true;
        }
        return false;
    }

    public void resetEncoder() {
        slideMotors[0].resetEncoder();
    }

    public void timeOut() {
        controller.timeOut();
    }

    @Override
    public void periodic() {
        if (usingPID.getAsBoolean()) {
            int position = slideMotors[0].getCurrentPosition();
            double power = controller.getPower(position);
            for (int i = 0; i < 2; i++) {
                slideMotors[i].set(power);
            }
            t.addData("Target Position", targetHeight);
            t.addData("Current Position", position);
        }

        t.update();


    }
}
