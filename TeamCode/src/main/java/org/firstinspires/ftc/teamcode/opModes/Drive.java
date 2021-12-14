package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class  Drive extends OpMode {
    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorMiddleLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    DcMotor motorMiddleRight;
    Boolean armCheck = false;
    Boolean spinToggle = false;
    DcMotor arm;
    DcMotor intake;
    Servo grabber;
    CRServo spinner;
    double position = 0.0; // Change Value
    double armPower = 0.1;
    double spinPower = 0.5;


    @Override
    public void init() {
        motorFrontLeft = hardwareMap.dcMotor.get("TLM10");
        motorBackLeft = hardwareMap.dcMotor.get("BLM11");
        motorFrontRight = hardwareMap.dcMotor.get("TRM12");
        motorBackRight = hardwareMap.dcMotor.get("BRM13");
        arm = hardwareMap.dcMotor.get("arm");
        grabber = hardwareMap.servo.get("grabber");
        spinner = hardwareMap.crservo.get("blueSpinner");
        intake = hardwareMap.dcMotor.get("intake23");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
//        MOEvement();
        intake();
        spinCarousel();
        drive();
    }
    public void drive(){
        motorFrontLeft.setPower(-gamepad1.left_stick_y * 0.5);
        motorBackLeft.setPower(-gamepad1.left_stick_y * 0.5);
        motorFrontRight.setPower(gamepad1.right_stick_y * 0.5);
        motorBackRight.setPower(gamepad1.right_stick_y * 0.5);
    }
    public void MOEvement() {
        double drivePower = -gamepad1.left_stick_y;
        double rotationPower = gamepad1.right_stick_x;

        motorFrontLeft.setPower(drivePower + rotationPower);
        motorFrontRight.setPower(drivePower - rotationPower);
        motorBackLeft.setPower(drivePower + rotationPower);
        motorBackRight.setPower(drivePower - rotationPower);

//        motorFrontLeft.setPower(-gamepad1.left_stick_y);
//        motorFrontRight.setPower(-gamepad1.left_stick_y);
//        motorBackLeft.setPower(-gamepad1.right_stick_y);
//        motorBackRight.setPower(-gamepad1.right_stick_y);
    }

    public void intake() {
        if(gamepad1.x){
            intake.setPower(1.0);
        } else{
            intake.setPower(0.0);
        }
    }

    public void spinCarousel() {
        if (gamepad1.a && !spinToggle) {
            spinner.setPower(spinPower);
        } else if (gamepad1.a) {
            spinner.setPower(0.0);
            spinToggle = !spinToggle;
        }
    }
}