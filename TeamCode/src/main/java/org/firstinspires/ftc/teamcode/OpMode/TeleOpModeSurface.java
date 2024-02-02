package org.firstinspires.ftc.teamcode.OpMode;

//Packages used
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Subsystems.Claw;
import org.firstinspires.ftc.teamcode.Subsystems.ClawHolder;
import org.firstinspires.ftc.teamcode.Subsystems.Elevator;
import org.firstinspires.ftc.teamcode.Subsystems.Arm;
import org.firstinspires.ftc.teamcode.Subsystems.AirplaneLancher;
import org.firstinspires.ftc.teamcode.Subsystems.Constants;

@TeleOp(name = "Mecanum Drive")
public class TeleOpModeSurface extends LinearOpMode {
    //Initialized motors for driving
    DcMotorEx topLeftMotor, topRightMotor, bottomLeftMotor, bottomRightMotor;

    //Subsystem classes
    Elevator elevator = new Elevator();
    Claw claw = new Claw();
    ClawHolder clawHolder = new ClawHolder();
    Arm armLeveller = new Arm();
    AirplaneLancher airplaneLancher = new AirplaneLancher();
    Constants constants = new Constants();

    public void runOpMode(){
        //Assign the motors and initialize the classes. Reverse the two right motors to get it working and also initialize telemetry.
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

        airplaneLancher.init((hardwareMap));

        waitForStart();

        //Start the robot off with the claw open and the claw holder on the ground. Update telemetry info

        claw.open();
        clawHolder.reset();

        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) {

            armLeveller.loop();

            //Telemetry for the robot. Used for testing and tuning.
            telemetry.addData("Front Left: ", topLeftMotor.getPower());
            telemetry.addData("Front Right: ", topRightMotor.getPower());
            telemetry.addData("Back Left: ", bottomLeftMotor.getPower());
            telemetry.addData("Back Right: ", bottomRightMotor.getPower());
            telemetry.addData("ElevMotLeft: ", elevator.LeftMot.getPower());
            telemetry.addData("ElevMotRight: ", elevator.RightMot.getPower());
            telemetry.addData("ArmMotLeft: ", armLeveller.armTwo.getPower());
            telemetry.addData("ArmMotRight: ", armLeveller.armOne.getPower());
            telemetry.update();

            /* drive is for forward/backward
           strafe is for left/right
           twist is to turn */
            double drive = gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double twist = gamepad1.right_stick_x;

            //This gives us the speed for the various motors. Multiply to lower the speeds.
            double[] speed = {
                    ((drive * 0.8) - (strafe * 0.8) - (twist * 0.7)),
                    ((drive * 0.8) + (strafe * 0.8) + (twist * 0.7)),
                    ((drive * 0.8) + (strafe * 0.8) - (twist * 0.7)),
                    ((drive * 0.8) - (strafe * 0.8) + (twist * 0.7))};

            //Calculate the maximum/largest speed of all the motors
            double max = Math.abs(speed[0]);

            for (int i = 0; i < speed.length; i++) {
                if(max < Math.abs(speed[i])) {
                    max = Math.abs(speed[i]);
                }
            }

            //If a motors power is above 1, divide it by the max to prevent more energy going to it.
            if(max > 1) {
                for (int i = 0; i < speed.length; i++) {
                    speed[i] /= max;
                }
            }

            //Set the powers.
            topLeftMotor.setPower(speed[0]);
            topRightMotor.setPower(speed[1]);
            bottomLeftMotor.setPower(speed[2]);
            bottomRightMotor.setPower(speed[3]);

//**************************************************************************************************************************************************************************************************************************************************

            //Close the claw to pick up pixels
            if(gamepad1.a || gamepad2.a){
                claw.close();
            }
            //Open the claw to drop pixels
            if(gamepad1.b || gamepad2.b){
                claw.open();
            }

//**************************************************************************************************************************************************************************************************************************************************

            //Rotate the claw over in order to place the pixels on the backdrop
            if(gamepad1.y){
                clawHolder.rotate();
            }
            //Reset the claw back onto the floor.
            if(gamepad1.x){
                clawHolder.reset();
            }

//**************************************************************************************************************************************************************************************************************************************************

            //Lift the arm up
            if(gamepad1.right_bumper){
                armLeveller.setTargetPos(525);
            }
            //Reset the arm onto the ground.
            if(gamepad1.right_trigger > 0.3){
                armLeveller.setTargetPos(0);
            }

//**************************************************************************************************************************************************************************************************************************************************

            //Launch airplane
            if(gamepad1.dpad_right){
                airplaneLancher.launch();
            }

//**************************************************************************************************************************************************************************************************************************************************

            //Automatically lift the elevator to max height
            if(gamepad1.dpad_up){
                elevator.moveLift(Constants.upDownStates.up, 1770);
            }

            //Automatically lower the elevator to minimum height
            if(gamepad1.dpad_down){
                elevator.moveLift(Constants.upDownStates.down, 0);
            }

            //Manually lift the elevator until it's max height
            if(gamepad1.left_bumper){
                if(height <= 1770){
                    elevator.moveLift(Constants.upDownStates.up, height + 100);
                    height += 13;
                    telemetry.update();
                }
            }
            //Manually lower the elevator until it's minimum height
            if(gamepad1.left_trigger > 0.3){
                if(height >= 0){
                    elevator.moveLift(Constants.upDownStates.down, height - 100);
                    height -= 13;
                    telemetry.update();
                }
            }

            //Set the height.
            height = elevator.getPosition();
        }
    }
}
