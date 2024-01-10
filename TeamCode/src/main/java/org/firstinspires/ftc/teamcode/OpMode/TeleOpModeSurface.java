package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.Constants;

@TeleOp(name = "Mecanum Drive Test Copy")
public class TeleOpModeSurface extends LinearOpMode {
    public DcMotorEx topLeftMotor, topRightMotor, bottomLeftMotor, bottomRightMotor;

    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        topLeftMotor = hardwareMap.get(DcMotorEx.class, "topLeft");
        topRightMotor = hardwareMap.get(DcMotorEx.class, "topRight");
        bottomLeftMotor = hardwareMap.get(DcMotorEx.class, "bottomLeft");
        bottomRightMotor = hardwareMap.get(DcMotorEx.class, "bottomRight");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Front Left: ", topLeftMotor.getPower());
            telemetry.addData("Front Right: ", topRightMotor.getPower());
            telemetry.addData("Back Left: ", bottomLeftMotor.getPower());
            telemetry.addData("Back Right: ", bottomRightMotor.getPower());
            telemetry.update();
            /* drive is for forward/backward
           strafe is for left/right
           twist is to turn
         */
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;

            //This gives us the speed for the various motors.
            double[] speed = {
                    (drive + strafe + twist),
                    (drive - strafe - twist),
                    (drive - strafe + twist),
                    (drive + strafe - twist)};

            //Calculate the maximum/largest speed of all the motors
            double max = Math.abs(speed[0]);

            for (double s : speed) {
                if(s > max) {
                    max = Math.abs(s);
                }
            }

            if(max > 1) {
                for (int i = 0; i < speed.length; i++) {
                    speed[i] /= max;
                }
            }

            topLeftMotor.setPower(-speed[0]);
            topRightMotor.setPower(-speed[1]);
            bottomLeftMotor.setPower(-speed[2]);
            bottomRightMotor.setPower(-speed[3]);
        }
    }
}
