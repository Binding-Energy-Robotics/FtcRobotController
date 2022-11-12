package org.firstinspires.ftc.teamcode.Framework.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.TeleDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class MecDrive extends CommandBase {

    private final TeleDrive drive;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;
    private final BooleanSupplier slowMode;


    public MecDrive(TeleDrive drive, DoubleSupplier forward, DoubleSupplier strafe,
                    DoubleSupplier turn, BooleanSupplier slowMode){
        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        this.slowMode = slowMode;
        addRequirements(drive);
    }

    public MecDrive(TeleDrive drive, DoubleSupplier forward,
                    DoubleSupplier strafe, DoubleSupplier turn) {
        this(drive, forward, strafe, turn, () -> false);
    }

    @Override
    public void execute(){
        drive.driveWithMultiplier(strafe.getAsDouble(), forward.getAsDouble(),
                turn.getAsDouble(), slowMode.getAsBoolean() ? 0.33 : 1.0);
    }
}
