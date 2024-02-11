package org.firstinspires.ftc.teamcode.OpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Cam;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;


public class CamTest extends LinearOpMode {
    Cam camera = new Cam();

    public void runOpMode(){
        camera.init(hardwareMap, Constants.cameraColor.red);

        int zone = 3;

        while (opModeInInit()) {
            zone = camera.getZone();
            telemetry.addData("Parking Zone: ", zone);
            telemetry.update();
        }

        waitForStart();

        telemetry.addData("Parking Zone: ", zone);
        telemetry.update();
    }
}
