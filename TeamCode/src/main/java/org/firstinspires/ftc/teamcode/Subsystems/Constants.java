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

    final double clawHolderRotate = 0.325;
    final double clawHolderReset = 0.98;
}
