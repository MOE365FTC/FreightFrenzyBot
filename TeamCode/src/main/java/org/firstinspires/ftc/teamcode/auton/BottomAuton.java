package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.net.CacheRequest;

public class BottomAuton extends LinearOpMode {
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    CRServo carousel;

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
        motorBackLeft = hardwareMap.dcMotor.get("BLM");
        motorBackRight = hardwareMap.dcMotor.get("BRM");
        carousel = hardwareMap.crservo.get("spinner");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        manualDrive(-0.5, 1.0); //drive backward to carousel

        carousel.setPower(0.5);
        hold(1.5);
        carousel.setPower(0.0);

        manualDrive(0.5, 0.4); //moves away from carousel
        manual90Turn(1); //1 for right turn, -1 for left turn
        manualDrive(1.0, 0.5);
        manual90Turn(-1);
        manualDrive(1.0, 3.5); //drive into warehouse

    }

    public void manualDrive(double power, double time){
        motorFrontLeft.setPower(power);
        motorFrontRight.setPower(power);
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);

        hold(time);

        motorFrontLeft.setPower(0.0);
        motorFrontRight.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorBackRight.setPower(0.0);
    }

    public void manual90Turn(int direction) { // 1 for turn right, -1 for turn left
        if(direction == 1 || direction == -1) {
            motorFrontLeft.setPower(0.7 * direction);
            motorFrontRight.setPower(-0.7 * direction);
            motorBackLeft.setPower(0.7 * direction);
            motorBackRight.setPower(-0.7 * direction);

            hold(0.8); //adjust so turn is 90 degrees

            motorFrontLeft.setPower(0.0);
            motorFrontRight.setPower(0.0);
            motorBackLeft.setPower(0.0);
            motorBackRight.setPower(0.0);
        }
    }

}
