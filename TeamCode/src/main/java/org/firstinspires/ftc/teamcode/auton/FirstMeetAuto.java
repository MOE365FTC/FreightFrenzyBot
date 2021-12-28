package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class FirstMeetAuto extends LinearOpMode {
    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    Boolean armCheck = false;
    Boolean spinToggle = false;
    DcMotorEx arm;
    Servo servoArm, outtake;
    DcMotor intake;
    Servo grabber;
    CRServo spinner;
    double armPower = 0.1;
    int armPos = 0;
    double spinPower = 0.5;

    ElapsedTime timer = new ElapsedTime();
    public void wait(double waitTime) {
        timer.reset();
        while(timer.time() < waitTime && opModeIsActive()) {
        }
    }

    void drive(double leftPower, double rightPower, double time){
        motorFrontLeft.setPower(leftPower);
        motorBackLeft.setPower(leftPower);
        motorBackRight.setPower(rightPower);
        motorFrontRight.setPower(rightPower);
        wait(time);
        motorFrontLeft.setPower(0.0);
        motorBackLeft.setPower(0.0);
        motorBackRight.setPower(0.0);
        motorFrontRight.setPower(0.0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        motorFrontLeft = hardwareMap.dcMotor.get("TLM10");
        motorBackLeft = hardwareMap.dcMotor.get("BLM11");
        motorFrontRight = hardwareMap.dcMotor.get("TRM12");
        motorBackRight = hardwareMap.dcMotor.get("BRM13");
        arm = hardwareMap.get(DcMotorEx.class, "arm20"); //arm
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setTargetPosition(0);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        servoArm = hardwareMap.get(Servo.class, "servoArm12");
//        grabber = hardwareMap.servo.get("grabber");
        outtake = hardwareMap.servo.get("out11");
        spinner = hardwareMap.crservo.get("blueSpinner");
        intake = hardwareMap.dcMotor.get("intake23");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servoArm.setPosition(0.0);
        waitForStart();

        drive(0.4,0.4,0.8);
        servoArm.setPosition(0.25);
        wait(3.0);
        servoArm.setPosition(0.0);
//        drive(-1.0,1.0,0.3);
//
//        drive(0.8,0.8,2.0); //Park in warehouse
//        servoArm.setPosition(0.0);

    }
}
