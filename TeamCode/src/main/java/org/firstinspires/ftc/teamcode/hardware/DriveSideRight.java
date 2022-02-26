package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DriveSideRight {

    public DcMotor front, back;
    Servo odoServo;

    double ticsPerInch;
    double backwardTicsPerInch;
    double odoUpPos;
    double odoDownPos;

    public DriveSideRight(DcMotor front, DcMotor back, Servo odoServo, boolean frontMotorReverse, boolean backMotorReverse, double odoUpPos, double odoDownPos, double ticsPerInch, double backwardTicsPerInch) {
        this.front = front;
        this.back = back;
        this.odoServo = odoServo;

        this.odoUpPos = odoUpPos;
        this.odoDownPos = odoDownPos;

        this.ticsPerInch = ticsPerInch;
        this.backwardTicsPerInch = backwardTicsPerInch;

        this.front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (frontMotorReverse)
            this.front.setDirection(DcMotor.Direction.REVERSE);
        if (backMotorReverse)
            this.back.setDirection(DcMotor.Direction.REVERSE);
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

    public double getPower(){
        return this.back.getPower();
    }

}