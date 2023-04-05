package org.firstinspires.ftc.teamcode.Framework.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Claw extends SubsystemBase {
    public static double GRAB_DISTANCE = 3;
    public static double RELEASE_DELAY = 0.5; // only grab if claw has been open for this long

    private HardwareMap hw;

    private ServoImplEx clawServo;
    private DigitalChannel beamSensor;
    private DistanceSensor clawSensor;

    private boolean servoClosed;
    private ElapsedTime openTime;

    public Claw(HardwareMap hw, String clawName, String beamName, String colorName){
        this.hw = hw;

        clawServo = hw.get(ServoImplEx.class, clawName);
        clawServo.setPwmRange(new PwmControl.PwmRange(500, 2500));

        beamSensor = hw.get(DigitalChannel.class, beamName);

        clawSensor = hw.get(DistanceSensor.class, colorName);

        openTime = new ElapsedTime();
        close();
    }

    public Claw(HardwareMap hw) {
        this(hw, "claw", "beamSensor", "clawSensor");
    }

    public void open(){
        clawServo.setPosition(0.5);
        servoClosed = false;
        openTime.reset();
    }

    public void close(){
        clawServo.setPosition(0);
        servoClosed = true;
    }

    public void toggle() {
        if (servoClosed) {
            open();
        } else {
            close();
        }
    }

    public boolean isBeamBroken() {
        return !beamSensor.getState(); // adafruit 3mm ir beam break sensor is active low
    }

    public boolean isConeDetected() {
        if (servoClosed)
            return false;

        if (openTime.seconds() < RELEASE_DELAY)
            return false;

        return clawSensor.getDistance(DistanceUnit.CM) <= GRAB_DISTANCE;
    }

    public boolean grabIfConeDetected() {
        boolean coneDetected = isConeDetected();

        if (coneDetected)
            close();

        return coneDetected;
    }
}
