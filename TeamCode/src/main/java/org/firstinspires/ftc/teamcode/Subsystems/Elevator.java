package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Elevator {
    public DcMotorEx LeftMot;
    public DcMotorEx RightMot;

    public void init(HardwareMap map){
        LeftMot = map.get(DcMotorEx.class, "elevMotorLeft");
        RightMot = map.get(DcMotorEx.class, "elevMotorRight");

        LeftMot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RightMot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        LeftMot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RightMot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveLift(){
        LeftMot.setTargetPosition(500);
        RightMot.setTargetPosition(-500);

        LeftMot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        RightMot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        LeftMot.setPower(-1);
        RightMot.setPower(1);
    }
}
