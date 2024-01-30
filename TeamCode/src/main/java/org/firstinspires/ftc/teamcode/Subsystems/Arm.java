package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class Arm {

    public DcMotorEx armOne;
    public DcMotorEx armTwo;

    public void init(HardwareMap hardwareMap){
        armOne = hardwareMap.get(DcMotorEx.class, "armOneMotor");
        armOne.setDirection(DcMotorSimple.Direction.REVERSE);
        armOne.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armOne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armTwo = hardwareMap.get(DcMotorEx.class, "armTwoMotor");
        armTwo.setDirection(DcMotorSimple.Direction.REVERSE);
        armTwo.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        armTwo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveLeveller(int height, double power){
        armOne.setTargetPosition(-height);
        armOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armOne.setPower(power);

        armTwo.setTargetPosition(height);
        armTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTwo.setPower(-power);
    }
}
