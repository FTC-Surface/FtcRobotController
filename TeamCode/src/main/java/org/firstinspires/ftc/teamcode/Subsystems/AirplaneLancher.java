package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AirplaneLancher {
    Constants constants = new Constants();

    public Servo airLaunch;

    public void init(HardwareMap hardwareMap){
        airLaunch = hardwareMap.get(Servo.class, "airplane");
    }

    public void launch(){
        airLaunch.setPosition(constants.airplaneLaunch);
    }

    public void reset(){
        airLaunch.setPosition(constants.airplaneReset);
    }
}
