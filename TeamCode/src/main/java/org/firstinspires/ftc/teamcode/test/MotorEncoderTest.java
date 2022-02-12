package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class MotorEncoderTest extends OpMode {
    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    DcMotor armMotor;
    boolean aPressed = false;
    @Override
    public void init() {
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "FLM10");
        motorBackLeft = hardwareMap.get(DcMotorEx.class, "BLM11");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "FRM00");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "BRM01");
        armMotor = hardwareMap.get(DcMotorEx.class, "TSM13");
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop(){
        telemetry.addData("frontLeft", motorFrontLeft.getCurrentPosition());
        telemetry.addData("frontRight", motorFrontRight.getCurrentPosition());
        telemetry.addData("backLeft", motorBackLeft.getCurrentPosition());
        telemetry.addData("backRight", motorBackRight.getCurrentPosition());
        telemetry.addData("arm", armMotor.getCurrentPosition());

        if(gamepad1.a && !aPressed){
            aPressed = true;
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        } else if(aPressed && !gamepad1.a){
            aPressed = false;
            motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }
}