package org.firstinspires.ftc.teamcode.Subsystems;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class AirPlaneLauncher {
    public Servo planeRel;
    public boolean isLaunch;

    public void init(HardwareMap map){
        planeRel = map.get(Servo.class, "planeServ");
    }

    public void launcher(Constants.launchStates states){
        switch(states){
            case launch:
                launchPlane();
                isLaunch = true;
                break;
            case reset:
                reset();
                isLaunch = false;
                break;
        }
    }

    public void launchPlane(){
        planeRel.setPosition(0.5);
    }

    public void reset(){
        planeRel.setPosition(0.08);
    }
}
