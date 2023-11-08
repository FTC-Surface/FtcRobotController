package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mecanum Drive Test Copy")
public class MotorTest extends OpMode {
    public DcMotor topLeftMotor = null;
    public DcMotor topRightMotor = null;
    public DcMotor bottomLeftMotor = null;
    public DcMotor bottomRightMotor = null;

    @Override
    public void init() {
        topLeftMotor = hardwareMap.get(DcMotor.class, "Top Left Motor");
        topRightMotor = hardwareMap.get(DcMotor.class, "Top Right Motor");
        bottomLeftMotor = hardwareMap.get(DcMotor.class, "Bottom Left Motor");
        bottomRightMotor = hardwareMap.get(DcMotor.class, "Bottom Right Motor");
    }

    @Override
    public void loop() {
        /* drive is for fordward/backward
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

        if(max > 1){
            for (int i = 0 ; i < speed.length ; i++){
                speed[i] /= max;
            }
        }

        topLeftMotor.setPower(speed[0]);
        topRightMotor.setPower(speed[1]);
        bottomLeftMotor.setPower(speed[2]);
        bottomRightMotor.setPower(speed[3]);
    }
}
