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

    final double rClawClose = 0.57;
    final double lClawClose = 0.05;
    final double rClawOpen = 0.36;
    final double lClawOpen = 0.3;

    final double rClawHolderRotate = 0.175;
    final double lClawHolderRotate = 0.275;
    final double rClawHolderReset = 0.815;
    final double lClawHolderReset = 0.93;
}
