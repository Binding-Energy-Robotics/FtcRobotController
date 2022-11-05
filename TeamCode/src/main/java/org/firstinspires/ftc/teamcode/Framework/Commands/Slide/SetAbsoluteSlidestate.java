package org.firstinspires.ftc.teamcode.Framework.Commands.Slide;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.Utilities.Direction;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideConstants;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideState;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

public class SetAbsoluteSlidestate extends CommandBase {
    private LinearSlide slide;
    private SlideState state;

    public SetAbsoluteSlidestate(LinearSlide slide, SlideState state){
        this.slide = slide;
        this.state = state;
    }

    @Override
    public void execute(){
        this.slide.setState(state);
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}

