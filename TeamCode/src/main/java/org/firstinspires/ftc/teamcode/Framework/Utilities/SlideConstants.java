package org.firstinspires.ftc.teamcode.Framework.Utilities;

import android.transition.Slide;

import org.firstinspires.ftc.teamcode.Framework.subsystems.LinearSlide;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class SlideConstants {

    public static int STARTING_TICKS;
    public static int DOWN_TICKS;
    public static int LOW_TICKS;
    public static int MED_TICKS;
    public static int HIGH_TICKS;

    public static double KP = 0.1;
    public static double KI = 0.0;
    public static double KD = 0.0;
    public static double KF = 0.1;

    public SlideConstants(LinearSlide slide){
        STARTING_TICKS = slide.getEncoder();
        DOWN_TICKS = STARTING_TICKS;
        LOW_TICKS = STARTING_TICKS + 200;
        MED_TICKS = STARTING_TICKS + 400;
        HIGH_TICKS = STARTING_TICKS + 800;
    }

    public static int getTicks(SlideState state){
        switch(state){
            case DOWN:
                return DOWN_TICKS;
            case LOW:
                return LOW_TICKS;
            case MEDIUM:
                return MED_TICKS;
            case HIGH:
                return HIGH_TICKS;
        }
        return DOWN_TICKS;
    }

    private static SlideState[] stateArray = {SlideState.DOWN, SlideState.LOW, SlideState.MEDIUM, SlideState.HIGH};
    private static List<SlideState> states = Arrays.asList(stateArray);

    public static SlideState getNextState(SlideState state){
        int nextState = states.indexOf(state) + 1;
        if(nextState > states.size() - 1){
            nextState = 0;
        }
        return states.get(nextState);
    }

    public static SlideState getPreviousState(SlideState state){
        int nextState = states.indexOf(state) - 1;
        if(nextState < 0){
            nextState = states.size() -1;
        }
        return states.get(nextState);
    }
}
