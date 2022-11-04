package org.firstinspires.ftc.teamcode.Framework.Commands.Slide;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Framework.Utilities.Direction;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideConstants;
import org.firstinspires.ftc.teamcode.Framework.Utilities.SlideState;
import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

public class SetSlideState extends CommandBase {

    private LinearSlide slide;
    private Direction dir;

    public SetSlideState(LinearSlide slide, Direction dir){
        this.slide = slide;
        this.dir = dir;
    }

    @Override
    public void execute(){
        if(dir == Direction.UP) {
            slide.setState(SlideConstants.getNextState(slide.getState()));
        } else {
            slide.setState(SlideConstants.getPreviousState(slide.getState()));
        }
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
