package org.firstinspires.ftc.teamcode.Framework.Commands.Claw;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.Claw;

public class OpenClaw extends CommandBase {

    private Claw claw;
    private long startTime;

    public OpenClaw(Claw claw){
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void execute(){
        startTime = System.nanoTime();
        this.claw.open();
    }

    @Override
    public boolean isFinished(){
        return startTime + 0.5e9 < System.nanoTime();
    }
}
