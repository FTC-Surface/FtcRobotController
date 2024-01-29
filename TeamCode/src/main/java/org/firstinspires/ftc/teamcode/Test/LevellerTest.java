package org.firstinspires.ftc.teamcode.Test;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawHolder;

@TeleOp(name = "Leveller Test Op")
@Config
public class LevellerTest extends OpMode{
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int targetPos = 0;

    private final double ticks_in_degree = 537.7;

    DcMotorEx arm;

    Claw claw = new Claw();
    ClawHolder clawHolder = new ClawHolder();

    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm = hardwareMap.get(DcMotorEx.class, "levellerMotor");
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        claw.init(hardwareMap);
        clawHolder.init(hardwareMap);

        claw.close();
        clawHolder.rotate();
    }

    @Override
    public void loop(){
        controller.setPID(p ,i ,d);
        int armPos = arm.getCurrentPosition();
        double pid = controller.calculate(armPos, targetPos);
        double ff = Math.cos(Math.toRadians(targetPos / ticks_in_degree)) * f;

        double power = pid + ff;

        arm.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", targetPos);
        telemetry.update();
    }
}
