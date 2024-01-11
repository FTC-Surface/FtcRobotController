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

    public void moveLift(Constants.elevStates state){
        switch (state){
            case up:
                LeftMot.setTargetPosition(-3000);
                RightMot.setTargetPosition(3000);

                LeftMot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                RightMot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                LeftMot.setPower(1);
                RightMot.setPower(-1);
                break;
            case down:
                LeftMot.setTargetPosition(-50);
                RightMot.setTargetPosition(50);

                LeftMot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                RightMot.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

                LeftMot.setPower(-1);
                RightMot.setPower(1);
        }

    }
}
