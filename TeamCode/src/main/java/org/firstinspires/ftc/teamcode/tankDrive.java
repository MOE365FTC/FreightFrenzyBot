package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp
public class tankDrive extends OpMode {

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;

    @Override
    public void init() {
        motorFrontLeft = hardwareMap.dcMotor.get("FLM");
        motorBackLeft = hardwareMap.dcMotor.get("BLM");
        motorFrontRight = hardwareMap.dcMotor.get("FRM");
        motorBackRight = hardwareMap.dcMotor.get("BRM");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        motorFrontLeft.setPower(-gamepad1.left_stick_y);
        motorBackLeft.setPower(-gamepad1.left_stick_y);
        motorFrontRight.setPower(-gamepad1.right_stick_y);
        motorBackRight.setPower(-gamepad1.right_stick_y);
    }

}
