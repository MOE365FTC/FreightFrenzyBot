package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class DriveEncoder extends OpMode {
    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    DcMotorEx arm;
    Servo servoArm, outtake, grabber;
    DcMotor intake;
    CRServo spinner;
    double armPower = 0.3;
    int armPos = 0;
    double spinPower = 1.0;
    final double leftScalar = 38.1, rightScalar = -37.25; // ticks per inch
    final double leftTurn = 5.29, rightTurn = 5.422; //ticks per degree
    @Override
    public void init() {
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "TLM10");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "BLM11");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "TRM12");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "BRM13");
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm = hardwareMap.get(DcMotorEx.class, "arm20"); //arm
        servoArm = hardwareMap.get(Servo.class, "servoArm12");
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
    }

    @Override
    public void loop() {

    }
}
