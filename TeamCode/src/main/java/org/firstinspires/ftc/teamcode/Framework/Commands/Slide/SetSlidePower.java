package org.firstinspires.ftc.teamcode.Framework.Commands.Slide;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

import java.util.function.DoubleSupplier;

public class SetSlidePower extends CommandBase {
    private LinearSlide slide;
    private DoubleSupplier power;
    public SetSlidePower(LinearSlide slide, DoubleSupplier power){
        this.slide = slide;
        this.power = power;
        addRequirements(slide);
    }

    @Override
    public void execute(){
        slide.setPower(power.getAsDouble());
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
