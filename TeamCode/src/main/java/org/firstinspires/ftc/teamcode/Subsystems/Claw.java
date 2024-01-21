package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {

    public Servo clawR;
    public Servo clawL;

    public void init(HardwareMap hardwareMap){
        clawR = hardwareMap.get(Servo.class, "rightClaw");
        clawL = hardwareMap.get(Servo.class, "leftClaw");
    }

    public void open(){
        clawR.setPosition(1);
        clawL.setPosition(1);
    }

    public void close(){
        clawR.setPosition(0);
        clawL.setPosition(0);
    }
}
