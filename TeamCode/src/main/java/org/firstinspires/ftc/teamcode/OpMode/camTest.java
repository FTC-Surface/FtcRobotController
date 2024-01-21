package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Cam;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;

@Autonomous(name = "CamTest")
public class camTest extends LinearOpMode {
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
