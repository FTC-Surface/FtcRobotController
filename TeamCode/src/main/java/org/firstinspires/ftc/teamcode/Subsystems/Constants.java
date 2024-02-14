package org.firstinspires.ftc.teamcode.Subsystems;

public class Constants {
    public enum autoStates{
        idle,
        ready,
        movedToBoard,
        spline,
        forward,
        board,
        reset,
        park
    }

    public enum upDownStates{
        up,
        down
    }

    public enum cameraColor{
        red,
        blue
    }

    public final double rClawClose = 0.71;
    public final double lClawClose = 0.05;
    public final double rClawOpen = 0.65;
    public final double lClawOpen = 0.515;

    public final double clawHolderRotate = 0;
    public final double clawHolderReset = 0.68;

    public final double airplaneLaunch = 0.2;
}
