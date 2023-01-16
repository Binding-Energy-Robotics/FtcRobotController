package org.firstinspires.ftc.teamcode.Framework.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Framework.subsystems.TeleDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MecDrive extends CommandBase {

    private final TeleDrive drive;
    private final BNO055IMU imu;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;
    private final BooleanSupplier slowMode;
    private final Telemetry t;
    private double initialAngle;

    public MecDrive(HardwareMap hw, TeleDrive drive, DoubleSupplier forward, DoubleSupplier strafe,
                    DoubleSupplier turn, BooleanSupplier slowMode, Telemetry t){
        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.slowMode = slowMode;
        this.t = t;
        imu = hw.get(BNO055IMU.class,"imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
        initialAngle = imu.getAngularOrientation().firstAngle;
        addRequirements(drive);
    }

    public MecDrive(HardwareMap hw, TeleDrive drive, DoubleSupplier forward,
                    DoubleSupplier strafe, DoubleSupplier turn, Telemetry t) {
        this(hw, drive, forward, strafe, turn, () -> false, t);
    }

    @Override
    public void execute() {
        drive.driveWithMultiplier(strafe.getAsDouble(), forward.getAsDouble(),
                turn.getAsDouble(), Math.toDegrees(imu.getAngularOrientation().firstAngle - initialAngle),
                slowMode.getAsBoolean() ? 0.33 : 1.0);
    }
}
