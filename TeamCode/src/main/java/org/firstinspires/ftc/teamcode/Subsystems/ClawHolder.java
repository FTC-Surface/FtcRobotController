package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawHolder {
    Constants constants = new Constants();

    public Servo clawHolderR;
    public Servo clawHolderL;

    public void init(HardwareMap hardwareMap){
        clawHolderR = hardwareMap.get(Servo.class, "rightClawHolder");
        clawHolderL = hardwareMap.get(Servo.class, "leftClawHolder");

        clawHolderR.setDirection(Servo.Direction.REVERSE);
    }

    public void rotate(){
        //clawHolderR.setPosition(constants.rClawHolderRotate);
        clawHolderL.setPosition(constants.lClawHolderRotate);
    }

    public void reset(){
        //clawHolderR.setPosition(constants.rClawHolderReset);
        clawHolderL.setPosition(constants.lClawHolderReset);
    }
}
