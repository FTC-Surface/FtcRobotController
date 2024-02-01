package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawHolder {
    Constants constants = new Constants();

    public Servo clawHolder;

    public void init(HardwareMap hardwareMap){
        clawHolder = hardwareMap.get(Servo.class, "rightClawHolder");
        clawHolder.setDirection(Servo.Direction.REVERSE);
    }

    public void rotate(){
        clawHolder.setPosition(constants.clawHolderRotate);
    }

    public void reset(){
        clawHolder.setPosition(constants.clawHolderReset);
    }
}
