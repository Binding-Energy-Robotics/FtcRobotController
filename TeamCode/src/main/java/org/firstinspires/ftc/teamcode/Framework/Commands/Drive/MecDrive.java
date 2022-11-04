package org.firstinspires.ftc.teamcode.Framework.Commands.Drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.TeleDrive;

import java.util.function.DoubleSupplier;

public class MecDrive extends CommandBase {

    private final TeleDrive drive;
    private final DoubleSupplier forward;
    private final DoubleSupplier strafe;
    private final DoubleSupplier turn;


    public MecDrive(TeleDrive drive, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier turn){
        this.drive = drive;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        addRequirements(drive);
    }

    @Override
    public void execute(){
        drive.drive(forward.getAsDouble(), strafe.getAsDouble(), turn.getAsDouble());
    }
}
