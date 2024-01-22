package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;

public class Claw {

    Constants constants = new Constants();

    public Servo clawR;
    public Servo clawL;

    public void init(HardwareMap hardwareMap){
        clawR = hardwareMap.get(Servo.class, "rightClaw");
        clawL = hardwareMap.get(Servo.class, "leftClaw");
    }

    public void close(){
        clawR.setPosition(constants.rClawClose);
        clawL.setPosition(constants.lClawClose);
    }

    public void open(){
        clawR.setPosition(constants.rClawOpen);
        clawL.setPosition(constants.lClawOpen);
    }
}
