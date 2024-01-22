package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;

//Name Courtesy of Kofi
public class Leveller {

    DcMotorEx levellerMotor;

    public void init(HardwareMap hardwareMap){
        levellerMotor = hardwareMap.get(DcMotorEx.class, "levellerMotor");
        levellerMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        levellerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        levellerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveLeveller(int height){
        levellerMotor.setTargetPosition(-height);
        levellerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        levellerMotor.setPower(0.2);
    }
}
