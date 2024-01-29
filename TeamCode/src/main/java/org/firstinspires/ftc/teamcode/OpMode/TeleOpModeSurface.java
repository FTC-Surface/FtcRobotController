package org.firstinspires.ftc.teamcode.OpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawHolder;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;

import org.firstinspires.ftc.teamcode.Subsystems.Constants;

/*
 ExpansionHub
 backRight is 0 (deadwheelRight goes to 0)
 frontRight is 1
 arm is 2
 rightLift is 3

 ControlHub
 backLeft is 0 (deadwheelLeft goes to 0)
 frontLeft goes to 1 (deadwheelLater goes to 1)
 leftElevator got to 2

 */

@TeleOp(name = "Mecanum Drive")
public class TeleOpModeSurface extends LinearOpMode {
    DcMotorEx topLeftMotor, topRightMotor, bottomLeftMotor, bottomRightMotor;
    Elevator elevator = new Elevator();

    Claw claw = new Claw();

    ClawHolder clawHolder = new ClawHolder();

    Arm armLeveller = new Arm();

    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        topLeftMotor = hardwareMap.get(DcMotorEx.class, "topLeft");
        topRightMotor = hardwareMap.get(DcMotorEx.class, "topRight");
        bottomLeftMotor = hardwareMap.get(DcMotorEx.class, "bottomLeft");
        bottomRightMotor = hardwareMap.get(DcMotorEx.class, "bottomRight");

        topLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        topRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bottomRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bottomRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        topRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        elevator.init(hardwareMap);
        int height = 0;

        claw.init(hardwareMap);

        armLeveller.init(hardwareMap);

        clawHolder.init(hardwareMap);

        waitForStart();

        claw.open();
        clawHolder.reset();

        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Front Left: ", topLeftMotor.getPower());
            telemetry.addData("Front Right: ", topRightMotor.getPower());
            telemetry.addData("Back Left: ", bottomLeftMotor.getPower());
            telemetry.addData("Back Right: ", bottomRightMotor.getPower());
            telemetry.addData("ElevMotLeft: ", elevator.LeftMot.getCurrentPosition());
            telemetry.addData("ElevMotRight: ", elevator.RightMot.getCurrentPosition());
            telemetry.addData("Height", height);
            telemetry.update();

            /* drive is for forward/backward
           strafe is for left/right
           twist is to turn */
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;

            //This gives us the speed for the various motors.
            double[] speed = {
                    ((drive * 0.85) - (strafe * 0.85) - (twist * 0.7)),
                    ((drive * 0.85) + (strafe * 0.85) + (twist * 0.7)),
                    ((drive * 0.85) + (strafe * 0.85) - (twist * 0.7)),
                    ((drive * 0.85) - (strafe * 0.85) + (twist * 0.7))};

            //Calculate the maximum/largest speed of all the motors
            double max = Math.abs(speed[0]);

            for (int i = 0; i < speed.length; i++) {
                if(max < Math.abs(speed[i])) {
                    max = Math.abs(speed[i]);
                }
            }

            if(max > 1) {
                for (int i = 0; i < speed.length; i++) {
                    speed[i] /= max;
                }
            }

            topLeftMotor.setPower(speed[0]);
            topRightMotor.setPower(speed[1]);
            bottomLeftMotor.setPower(speed[2]);
            bottomRightMotor.setPower(speed[3]);

//**************************************************************************************************************************************************************************************************************************************************

            if(gamepad1.a){
                claw.close();
            }
            if(gamepad1.b){
                claw.open();
            }

//**************************************************************************************************************************************************************************************************************************************************

            if(gamepad1.x){
                clawHolder.rotate();
            }
            if(gamepad1.y){
                clawHolder.reset();
            }

//**************************************************************************************************************************************************************************************************************************************************

            if(gamepad1.right_bumper){
                armLeveller.moveLeveller(600, 0.8);
            }

            if(gamepad1.right_trigger > 0.3){
                armLeveller.moveLeveller(10, 0.2);
            }

//**************************************************************************************************************************************************************************************************************************************************

            if(gamepad1.dpad_up){
                elevator.moveLift(Constants.upDownStates.up, 1770);
            }
            if(gamepad1.dpad_down){
                elevator.moveLift(Constants.upDownStates.down, 0);
            }

            if(gamepad1.left_bumper){
                if(height <= 1770){
                    elevator.moveLift(Constants.upDownStates.up, height + 100);
                    height += 13;
                    telemetry.update();
                }
            }
            if(gamepad1.left_trigger > 0.3){
                if(height >= 0){
                    elevator.moveLift(Constants.upDownStates.down, height - 100);
                    height -= 13;
                    telemetry.update();
                }
            }

            height = elevator.getPosition();
        }
    }
}
