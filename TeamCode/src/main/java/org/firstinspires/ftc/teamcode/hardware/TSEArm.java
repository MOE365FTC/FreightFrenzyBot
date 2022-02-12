package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.TSEArmPos;

@Config
public class TSEArm {
    DcMotorEx arm;
    Servo grabber;
    Gamepad gamepad1;
    LinearOpMode opMode;

    TSEArmPos armSetting = TSEArmPos.RETRACTED;

    private final int RETRACT_POS = 110;
    private final int GRAB_POS = 945;
    private final int preDROP_POS = 490;
    private final int DROP_POS = 580;

    private final double open = 0.0;
    private final double close = 1.0;

    private final double ARM_POWER = 0.8;
    private final double RESET_POWER = 0.4;
    public static double p = 3;
    public TSEArm(HardwareMap hardwareMap, Gamepad gpad1){
        this.gamepad1 = gpad1;

        arm = hardwareMap.get(DcMotorEx.class, "TSM13");
        grabber = hardwareMap.get(Servo.class, "TGS04");

        arm.setPositionPIDFCoefficients(p);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
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
            this.grabber.setPosition(this.open);
        } else{
            this.grabber.setPosition(this.close);
        }

        if(this.armSetting == TSEArmPos.DROPPING){
            this.arm.setTargetPosition(this.DROP_POS);
        } else if (this.armSetting == TSEArmPos.GRABBING){
            this.arm.setTargetPosition(this.GRAB_POS);
        } else if(this.armSetting == TSEArmPos.PREDROPPING){
            this.arm.setTargetPosition(this.preDROP_POS);
        }
        else {
            this.arm.setTargetPosition(this.RETRACT_POS);
        }

        this.arm.setPower(this.ARM_POWER);
    }

    public void setArm(){
        if(gamepad1.b){
            this.armSetting = TSEArmPos.DROPPING;
        } else if (gamepad1.y){
            this.armSetting = TSEArmPos.PREDROPPING;
        } else if(gamepad1.x){
            this.armSetting = TSEArmPos.RETRACTED;
        } else if(gamepad1.a){
            this.armSetting = TSEArmPos.GRABBING;
        }
    }

    public void autonGrab(){
        this.grabber.setPosition(this.close);

        this.armSetting = TSEArmPos.RETRACTED;
        this.arm.setTargetPosition(RETRACT_POS);
        this.arm.setPower(this.ARM_POWER);
    }

    public void autonDeploy(){
        this.grabber.setPosition(this.open);
        this.armSetting = TSEArmPos.GRABBING;
        this.arm.setTargetPosition(GRAB_POS);
        this.arm.setPower(this.ARM_POWER);
    }


    public void resetControl(){
        this.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(gamepad1.right_trigger > 0.5){
            this.arm.setPower(RESET_POWER);
        } else if(gamepad1.left_trigger > 0.5){
            this.arm.setPower(-RESET_POWER);
        } else{
            this.arm.setPower(0);
        }
    }
    public void composeTelemetry(Telemetry telemetry){
        telemetry.addLine("--TSE ARM--");
        telemetry.addData("Setting", this.armSetting);
        telemetry.addData("Pos", this.arm.getCurrentPosition());
        telemetry.addData("Target", this.arm.getTargetPosition());
        telemetry.addData("Power", this.arm.getPower());
    }
}
