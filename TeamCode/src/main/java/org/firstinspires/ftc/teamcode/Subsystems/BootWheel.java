package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BootWheel {

    public DcMotorEx bootWheel;

    public void init(HardwareMap hardwareMap){
        bootWheel = hardwareMap.get(DcMotorEx.class,"bootWheel");
    }

    public void startSpin(){
        bootWheel.setPower(0.5);
    }

    public void stopSpin(){
        bootWheel.setPower(0);
    }

    public void spinWheel(Constants.wheelState input){
        switch (input){
            case Spin:
                startSpin();
                break;

            case Stop:
                stopSpin();
                break;
        }
    }
}
