package org.firstinspires.ftc.teamcode.hardware;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.SlideSetting;
import org.firstinspires.ftc.teamcode.enums.SlideState;

@Config
public class Slides {
    AnalogInput limitSwitch0 ,limitSwitch1;
    DcMotorEx slideExtend, slideRotate;
    Gamepad gamepad2;
    LinearOpMode opMode;

    public SlideState curSlideState = SlideState.RETRACTED;
    public SlideSetting curSlideSetting = SlideSetting.RETRACT;
    public int slideTargetPos = 0;
    public double slideTiltInput = 0;

    public final double SLIDE_RETRACT_POWER = -0.5;
    public final double SLIDE_EXTEND_POWER = 0.8;
    public final double tiltTicsFor90degrees = 2700.0; //Number of tics for 90 degrees of slide rotation (influences dispenser tilt)
    public final double slideTiltScale = 0.75;
    public final double slideTiltMax = tiltTicsFor90degrees*slideTiltScale;
    public final double slideTiltMin = 377; //amount of tics for slides to be vertical relative to tilted starting position    final int extendMinimum = 263; //Min slide extension tics before tilting dispenser or opening gate (must be above motor to prevent damage) (used for tilt and gate)
    public final double rotateTicsDeltaToVertical = 1180.0;
    final int extendMinimum = 263; //Min slide extension tics before tilting dispenser or opening gate (must be above motor to prevent damage) (used for tilt and gate)

    private boolean isTiltManual = false;
    private double SLIDE_TILT_MANUAL_POWER = -0.5;
    private double tiltPower = 0;

    public static  double tiltKp = 0.002;
    public static  double tiltKi = 0.00005;
    public static  double tiltKd = 0.0005;
    public static  int tiltTolerance = 30;
    public static  double integralCap = 100;
    protected int target = 0;

    public static  int tiltOut = 2200;
    public final int tiltReset = 500;
    public static  int sharedHubTilt = 1900;
    public PIDController tiltPID;

    public Slides(HardwareMap hardwareMap, Gamepad gpad2){
        this.gamepad2 = gpad2;

        limitSwitch0 = hardwareMap.get(AnalogInput.class, "SEA00");
        limitSwitch1 = hardwareMap.get(AnalogInput.class, "SRA01");

        slideExtend = hardwareMap.get(DcMotorEx.class, "SEM03");
        slideRotate = hardwareMap.get(DcMotorEx.class, "SRM02");

        slideExtend.setTargetPosition(0);
        slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        slideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // slideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        slideRotate.setTargetPosition(0);
//        slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideExtend.setDirection(DcMotor.Direction.REVERSE);
        slideRotate.setDirection(DcMotor.Direction.REVERSE);
        slideExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.tiltPID = new PIDController(slideRotate, tiltKp, tiltKi, tiltKd, tiltTolerance, integralCap);
        this.tiltPID.setTarget(this.getCurrentRotation()); //Set slide rotate target to the current position
        updateState();
    }

    public Slides(HardwareMap hardwareMap, Gamepad gpad2, LinearOpMode opMode){
        this(hardwareMap, gpad2);
        this.opMode = opMode;

        slideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideExtend.setTargetPosition(0);
        slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.tiltPID.setOpMode(opMode);
        this.tiltPID.setTarget(0);

        updateState();
    }

    public void actuate(){
        actuateExtension();
        actuateTilt();
    }
    public void setExtension(){
        if(gamepad2.a){
            curSlideSetting = SlideSetting.EXTEND;
            slideTargetPos = 790;
        } else if(gamepad2.b) {
            curSlideSetting = SlideSetting.EXTEND;
            slideTargetPos = 1580;
        } else if(gamepad2.x){
            curSlideSetting = SlideSetting.RETRACT;
            slideTargetPos = 0;
            tiltPID.setTarget(tiltReset);
        }
    }

    public void updateState(){
        if(limitSwitch0.getVoltage() > 1 && limitSwitch1.getVoltage() < 1){
            this.curSlideState = SlideState.EXTENDED;
        } else if(limitSwitch0.getVoltage() < 1 && limitSwitch1.getVoltage() > 1){
            this.curSlideState = SlideState.RETRACTED;
        } else {
//            telemetry.addLine("LIMIT SWITCH DISCONNECTED"); //TODO: Create telemetry class
            this.curSlideState = SlideState.ERROR;
        }
    }

    void actuateExtension() {
        updateState();
        if(curSlideState == SlideState.RETRACTED && slideExtend.getCurrentPosition() != 0 && curSlideSetting != SlideSetting.MANUAL_OVERRIDE){
            slideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(curSlideSetting == SlideSetting.RETRACT && curSlideState != SlideState.RETRACTED && curSlideSetting != SlideSetting.MANUAL_OVERRIDE) {
            slideExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideExtend.setPower(SLIDE_RETRACT_POWER);
        } else if(curSlideSetting == SlideSetting.EXTEND){
            slideExtend.setTargetPosition(slideTargetPos);
            slideExtend.setPower(SLIDE_EXTEND_POWER);
            slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else if(curSlideSetting == SlideSetting.MANUAL_OVERRIDE){
            slideExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideExtend.setPower(gamepad2.right_stick_y);
        } else{
            slideExtend.setPower(0);
        }
    }

    void actuateTilt(){
//        //We want GAMEPAD UP (negative value) to make the motor go OUT (negative value)
//        final double slideRotateScalar = -0.5;
//        slideTiltInput = gamepad2.left_stick_y;
//        if(slideRotate.getCurrentPosition() <= 0 && slideTiltInput > 0) // we're all the way back, and trying to go more back
//            slideTiltInput = 0;
//        else if(slideRotate.getCurrentPosition() >= slideTiltMax && slideTiltInput < 0) // we're all the way out and trying to go more out
//            slideTiltInput = 0;
//
//        slideRotate.setPower(slideTiltInput * slideRotateScalar);
        if(isTiltManual) {
            this.slideRotate.setPower(tiltPower);
        }
        else{
            tiltPID.moveTeleop();
        }

        if(gamepad2.right_stick_button){
            slideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setTilt(){
        if(-gamepad2.right_stick_y > 0.75) {
            this.isTiltManual = false;
            target = tiltOut;
        }
        else if(-gamepad2.right_stick_y < -0.75 && this.curSlideState == SlideState.RETRACTED) {
            this.isTiltManual = false;
            target = tiltReset;
        } else if(gamepad2.left_trigger > 0.9){
            this.isTiltManual = true;
            this.tiltPower = SLIDE_TILT_MANUAL_POWER;
        } else{
            this.tiltPower = 0;
        }

        tiltPID.setTarget(target);
    }

    public void autonSlideTilt(double speed, double target, int tolerance){
        int dir = 1; // assume we go forwards
        if(slideRotate.getCurrentPosition() > target){ // now we need to go backwards
            dir = -1;
        }
        int error = (int) Math.abs(slideRotate.getCurrentPosition() - target);
        slideRotate.setPower(dir * speed);
        while(error > tolerance && this.opMode.opModeIsActive()){
            error = (int) Math.abs(slideRotate.getCurrentPosition() - target);
        }
        slideRotate.setPower(0);
    }

    public void autonArm(int rotPos, int extPos){
        this.tiltPID.setTarget(rotPos);
        this.tiltPID.moveAuton();
//        this.autonSlideTilt(0.5, rotPos, 10);
        slideExtend.setTargetPosition(extPos);
        slideExtend.setPower(0.8);
    }

    public int getCurrentExtension(){
        return slideExtend.getCurrentPosition();
    }

    public int getCurrentRotation(){
        return slideRotate.getCurrentPosition();
    }

    public boolean isBusy(){
        return slideExtend.isBusy() || this.tiltPID.isBusy();
    }

    public void retractAndWait(){
        slideExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(curSlideState == SlideState.EXTENDED && opMode.opModeIsActive()){
            updateState();
            slideExtend.setPower(SLIDE_RETRACT_POWER);
        }
        slideExtend.setPower(0.0);
    }

    public void rotateRunToPosition(int targetPos, double power, double p, double i, double d, double f){
//        PIDFCoefficients pidf = new PIDFCoefficients(p, i, d, f);
//            slideRotate.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidf);
            slideRotate.setVelocityPIDFCoefficients(p, i, d, f);
            slideRotate.setTargetPosition(targetPos);
            slideRotate.setPower(power);
    }

    public void composeTelemetry(Telemetry telemetry){
        telemetry.addLine("--SLIDES--");
        telemetry.addData("SLIDE STATUS", curSlideState);
        telemetry.addData("EXT", this.getCurrentExtension());
        telemetry.addData("ROT", this.getCurrentRotation());
        telemetry.addData("SlideInput", gamepad2.left_stick_y);
        telemetry.addData("LS1", limitSwitch0.getVoltage());
        telemetry.addData("LS2", limitSwitch1.getVoltage());
        telemetry.addData("Rotate Power", this.tiltPID.power);
        telemetry.addData("Tilt Target", this.tiltPID.target);
    }
}
