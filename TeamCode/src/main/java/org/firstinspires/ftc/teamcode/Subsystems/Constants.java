package org.firstinspires.ftc.teamcode.Subsystems;

public class Constants {
    public enum autoStates{
        idle,
        ready,
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

    public final double rClawClose = 0.57;
    public final double lClawClose = 0.05;
    public final double rClawOpen = 0.36;
    public final double lClawOpen = 0.3;

    public final double clawHolderRotate = 0.33;
    public final double clawHolderReset = 0.6;

    public final double airplaneLaunch = 0.2;
}
