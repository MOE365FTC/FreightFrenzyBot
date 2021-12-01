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
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    Boolean armCheck = false;
    Boolean spinToggle = false;
    DcMotor arm;
    Servo grabber;
    CRServo spinner;
    double position = 0.0; // Change Value
    double armPower = 0.1;
    double spinPower = 0.5;


    @Override
    public void init() {
        motorFrontLeft = hardwareMap.dcMotor.get("FLM");
        motorBackLeft = hardwareMap.dcMotor.get("BLM");
        motorFrontRight = hardwareMap.dcMotor.get("FRM");
        motorBackRight = hardwareMap.dcMotor.get("BRM");
        arm = hardwareMap.dcMotor.get("arm");
        grabber = hardwareMap.servo.get("grabber");
        spinner = hardwareMap.crservo.get("spinner");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        MOEvement();
        arm();
        spinCarousel();
    }

    public void MOEvement() {
        // Below drive should work use the backup one (more basic) if it doesn't
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

    public void arm() {
        if (!gamepad1.dpad_up && gamepad1.dpad_down) {
            arm.setPower((gamepad1.dpad_up ? 1 : 0) * -armPower);

        } else if (gamepad1.dpad_up && !gamepad1.dpad_down) {
            arm.setPower((gamepad1.dpad_up ? 1 : 0) * armPower);

        }  else {
            arm.setPower(0);
        }

        if (gamepad1.a && !armCheck) {
            grabber.setPosition(position);
            armCheck = true;
        } else if (gamepad1.a) {
            grabber.setPosition(position);
            armCheck = !armCheck;
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