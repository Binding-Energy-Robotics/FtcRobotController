package org.firstinspires.ftc.teamcode.Framework.Commands.Claw;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;

public class CloseClaw extends InstantCommand {

    private Claw claw;
    private long startTime;

    public CloseClaw(Claw claw){
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void execute(){
        startTime = System.nanoTime();
        this.claw.close();
    }
}
