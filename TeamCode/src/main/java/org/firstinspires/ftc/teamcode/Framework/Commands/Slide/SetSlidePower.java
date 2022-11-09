package org.firstinspires.ftc.teamcode.Framework.Commands.Slide;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

import java.util.function.DoubleSupplier;

public class SetSlidePower extends CommandBase {
    private LinearSlide slide;
    private DoubleSupplier power;
    private boolean finished;
    public SetSlidePower(LinearSlide slide, DoubleSupplier power){
        this.slide = slide;
        this.power = power;
        finished = false;
        addRequirements(slide);
    }

    @Override
    public void execute(){
        if(slide.getEncoder() < slide.getStopValue() && power.getAsDouble() < 0){
            slide.setPower(0);
            finished = true;
        }
        else {
            slide.setPower(power.getAsDouble());
        }
    }

    @Override
    public boolean isFinished(){
        return finished;
    }

}
