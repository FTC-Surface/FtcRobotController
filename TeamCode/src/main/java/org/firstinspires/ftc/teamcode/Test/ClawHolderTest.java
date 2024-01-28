package org.firstinspires.ftc.teamcode.Test;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp(name = "Claw Holder Test Op")
@Config
public class ClawHolderTest extends LinearOpMode {
    Servo clawHolderR;
    Servo clawHolderL;
    public static double targetRight = 0.15;
    public static double targetLeft = 0.415;
    public void runOpMode() {

        clawHolderR = hardwareMap.get(Servo.class, "rightClawHolder");
        clawHolderL = hardwareMap.get(Servo.class, "leftClawHolder");

        clawHolderR.setDirection(Servo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            clawHolderR.setPosition(targetRight);
             clawHolderL.setPosition(targetLeft);
        }
    }
}