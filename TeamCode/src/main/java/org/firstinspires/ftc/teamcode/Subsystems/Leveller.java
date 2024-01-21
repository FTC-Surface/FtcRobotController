package org.firstinspires.ftc.teamcode.Subsystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Name Courtesy of Kofi
public class Leveller {

    DcMotorEx levellerMotor;

    public void init(HardwareMap hardwareMap){
        levellerMotor = hardwareMap.get(DcMotorEx.class, "levellerMotor");
    }
}
