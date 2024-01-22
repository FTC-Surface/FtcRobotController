package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawHolder {
    Constants constants = new Constants();

    public Servo clawHolderR;
    public Servo clawHolderL;

    public void init(HardwareMap hardwareMap){
        clawHolderR = hardwareMap.get(Servo.class, "RightClawHolder");
        clawHolderL = hardwareMap.get(Servo.class, "LeftClawHolder");
    }

    public void rotate(){
        clawHolderL.setPosition(constants.lClawHolderRotate);
        clawHolderR.setPosition(constants.rClawHolderRotate);
    }

    public void reset(){
        clawHolderL.setPosition(constants.lClawHolderReset);
        clawHolderR.setPosition(constants.rClawHolderReset);
    }
}
