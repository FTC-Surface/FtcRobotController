package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AirPlaneLauncher {
    public Servo planeRel;

    public void init(HardwareMap map){
        planeRel = map.get(Servo.class, "planeServ");
    }

    public void launchPlane(){
        planeRel.setPosition(0.5);
    }

    public void reset(){
        planeRel.setPosition(0);
    }
}
