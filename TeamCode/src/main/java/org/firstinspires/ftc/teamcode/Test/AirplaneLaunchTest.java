package org.firstinspires.ftc.teamcode.Test;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Airplane Test")
@Config
public class AirplaneLaunchTest extends LinearOpMode {
    Servo airLaunch;
    public static double value = 0;
    public void runOpMode() {

        airLaunch = hardwareMap.get(Servo.class, "airplane");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            airLaunch.setPosition(value);
        }
    }
}