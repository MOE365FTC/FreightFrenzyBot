package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DriveSide {

    DcMotor front, back;
    Servo odoServo;

    float ticsPerInch = 0;
    float backwardTicsPerInch = 0;
    double odoUpPos;
    double odoDownPos;

    public DriveSide(HardwareMap hardwareMap, String frontMotorName, boolean frontMotorReverse, String backMotorName,
                     boolean backMotorReverse, String odoServoName, double odoUpPos, double odoDownPos) {
        this.front = hardwareMap.get(DcMotor.class, frontMotorName);
        this.back = hardwareMap.get(DcMotor.class, backMotorName);
        this.odoServo = hardwareMap.get(Servo.class, odoServoName);

        this.odoUpPos = odoUpPos;
        this.odoDownPos = odoDownPos;

        this.front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (frontMotorReverse)
            this.front.setDirection(DcMotor.Direction.REVERSE);
        if (backMotorReverse)
            this.front.setDirection(DcMotor.Direction.REVERSE);
        this.front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        this.front.setPower(power);
        this.back.setPower(power);
    }

    public void stopDrive() {
        this.front.setPower(0);
        this.back.setPower(0);
    }

    public int getCurrentPosition() {
        return this.front.getCurrentPosition();
    }

    public void setOdometryDown(boolean isDown) {
        if (isDown)
            this.odoServo.setPosition(this.odoDownPos);
        else
            this.odoServo.setPosition(this.odoUpPos);
    }

    public void zeroEncoders(){
        this.front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}