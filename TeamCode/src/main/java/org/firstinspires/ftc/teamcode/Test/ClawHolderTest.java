package org.firstinspires.ftc.teamcode.Test;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Claw Holder Test Op")
@Config
public class ClawHolderTest extends LinearOpMode {
    Servo clawHolder;
    public static double targetRight = 0.15;
    public void runOpMode() {

        clawHolder = hardwareMap.get(Servo.class, "rightClawHolder");

        clawHolder.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            clawHolder.setPosition(targetRight);
        }
    }
}