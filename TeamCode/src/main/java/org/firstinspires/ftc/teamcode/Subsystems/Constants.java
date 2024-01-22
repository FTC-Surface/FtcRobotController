package org.firstinspires.ftc.teamcode.Subsystems;

public class Constants {
    public enum autoStates{
        idle,
        ready,
        forward,
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

    final double rClawClose = 0.5;
    final double lClawClose = 0.1;
    final double rClawOpen = 0.175;
    final double lClawOpen = 0.425;

    final double rClawHolderRotate = 1;
    final double lClawHolderRotate = 1;
    final double rClawHolderReset = 0;
    final double lClawHolderReset = 0;
}
