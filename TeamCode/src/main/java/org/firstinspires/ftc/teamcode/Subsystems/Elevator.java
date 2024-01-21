package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Elevator {
    public DcMotorEx LeftMot;
    public DcMotorEx RightMot;

    public void init(HardwareMap hardwareMap){
        LeftMot = hardwareMap.get(DcMotorEx.class, "elevMotorLeft");
        LeftMot.setDirection(DcMotorSimple.Direction.REVERSE);
        RightMot = hardwareMap.get(DcMotorEx.class, "elevMotorRight");
        RightMot.setDirection(DcMotorSimple.Direction.REVERSE);

        LeftMot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        RightMot.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        LeftMot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        RightMot.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void moveLift(Constants.elevStates state,int height){
        switch (state){
            case up:
                LeftMot.setTargetPosition(-height);
                RightMot.setTargetPosition(height);

                LeftMot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                RightMot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                LeftMot.setPower(1);
                RightMot.setPower(-1);
                break;
            case down:
                LeftMot.setTargetPosition(-height);
                RightMot.setTargetPosition(height);

                LeftMot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                RightMot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                LeftMot.setPower(-1);
                RightMot.setPower(1);
        }
    }

    public int getPosition() {
        return ((RightMot.getCurrentPosition() - LeftMot.getCurrentPosition())/2);
    }
}
