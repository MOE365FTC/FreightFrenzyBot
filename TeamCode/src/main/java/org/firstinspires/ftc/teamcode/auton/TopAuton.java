package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TopAuton extends LinearOpMode {
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorMiddleLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorMiddleRight;

    ElapsedTime timer = new ElapsedTime();
    public void hold(double waitTime) {
        timer.reset();
        while(timer.time() < waitTime && opModeIsActive()) {

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.dcMotor.get("FLM");
        motorFrontRight = hardwareMap.dcMotor.get("FRM");
        motorMiddleLeft = hardwareMap.dcMotor.get("MLM");
        motorBackLeft = hardwareMap.dcMotor.get("BLM");
        motorBackRight = hardwareMap.dcMotor.get("BRM");
        motorMiddleRight = hardwareMap.dcMotor.get("MRM");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        motorFrontLeft.setPower(1.0);
        motorFrontRight.setPower(1.0);
        motorMiddleLeft.setPower(1.0);
        motorBackLeft.setPower(1.0);
        motorBackRight.setPower(1.0);
        motorMiddleRight.setPower(1.0);

        hold(1.0);

        motorFrontLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorMiddleLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorBackRight.setPower(0.0);
        motorMiddleRight.setPower(0.0);
    }
}
