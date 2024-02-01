package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm{
    private PIDController controller;

    public double p = 0.0025, i = 0, d = 0.0001;
    public double fUp = 0.1;
    public double fDown = 0.065;

    public double pMult = 1;

    public int targetPos = 0;

    private final double ticks_in_degree = 537.7;

    public DcMotorEx armOne;
    public DcMotorEx armTwo;

    Constants.armState state;

    public void setTargetPos(int targetPos){
        this.targetPos = targetPos;
        this.state = state;
    }

    public void init(HardwareMap hardwareMap){
        controller = new PIDController(p, i, d);

        armOne = hardwareMap.get(DcMotorEx.class, "armOneMotor");

        armTwo = hardwareMap.get(DcMotorEx.class, "armTwoMotor");
    }

    public void loop() {
        controller.setPID(p, i, d);
        int armPos = armOne.getCurrentPosition();
        double pid = controller.calculate(armPos, targetPos);

        double ff = Math.cos(Math.toRadians(targetPos / ticks_in_degree)) * fUp;
        pMult = 1;

        if (targetPos == 0){
            ff = Math.sin(2 * Math.toRadians(targetPos / ticks_in_degree)) * fDown;
            pMult = 0.5;
        }

        double power = (pid + ff) * pMult;

        armOne.setPower(power);
        armTwo.setPower(-power);
    }
}
