package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.enums.TSEArmPos;

@Config
public class TSEArm {
    DcMotorEx arm;
    Servo leftGrabber, rightGrabber;
    Gamepad gamepad1;
    LinearOpMode opMode;

    TSEArmPos armSetting = TSEArmPos.RETRACTED;

    private final int RETRACT_POS = 0;
    private final int GRAB_POS = 0;
    private final int DROP_POS = 0;

    private final double leftOpen = 1.0;
    private final double leftClose = 0;
    private final double rightOpen = 0;
    private final double rightClose = 1.0;

    private final double ARM_POWER = 0.5;

    public TSEArm(HardwareMap hardwareMap, Gamepad gpad1){
        this.gamepad1 = gpad1;

        arm = hardwareMap.get(DcMotorEx.class, "TSM13");
        leftGrabber = hardwareMap.get(Servo.class, "LTS14");
        rightGrabber = hardwareMap.get(Servo.class, "RTS04");

        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setTargetPosition(RETRACT_POS);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public TSEArm(HardwareMap hardwareMap, Gamepad gpad1, LinearOpMode opMode){
        this(hardwareMap, gpad1);

        this.opMode = opMode;
    }

    public void actuate(){

        if(this.gamepad1.left_bumper){
            this.leftGrabber.setPosition(this.leftOpen);
        } else{
            this.leftGrabber.setPosition(this.leftClose);
        }

        if(this.gamepad1.right_bumper){
            this.rightGrabber.setPosition(this.rightOpen);
        } else {
            this.rightGrabber.setPosition(this.rightClose);
        }

        if(this.armSetting == TSEArmPos.DROPPING){
            this.arm.setTargetPosition(this.DROP_POS);
        } else if (this.armSetting == TSEArmPos.GRABBING){
            this.arm.setTargetPosition(this.GRAB_POS);
        } else {
            this.arm.setTargetPosition(this.RETRACT_POS);
        }

        this.arm.setPower(this.ARM_POWER);
    }

    public void setArm(){
        if(gamepad1.x){
            this.armSetting = TSEArmPos.DROPPING;
        } else if(gamepad1.y){
            this.armSetting = TSEArmPos.RETRACTED;
        } else if(gamepad1.b){
            this.armSetting = TSEArmPos.GRABBING;
        }
    }
}
