package org.firstinspires.ftc.teamcode.Test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Elevator Test Op")
@Config
public class ElevatorTest extends LinearOpMode {

    public DcMotorEx LeftMot;
    public DcMotorEx RightMot;

    public static double height = 100;

    @Override
    public void runOpMode(){

        LeftMot = hardwareMap.get(DcMotorEx.class, "elevMotorLeft");
        LeftMot.setDirection(DcMotorSimple.Direction.REVERSE);
        RightMot = hardwareMap.get(DcMotorEx.class, "elevMotorRight");
        RightMot.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftMot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RightMot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        LeftMot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RightMot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            LeftMot.setTargetPosition(-(int)height);
            RightMot.setTargetPosition((int)height);

            LeftMot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            RightMot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            LeftMot.setPower(1);
            RightMot.setPower(-1);
        }
    }
}
