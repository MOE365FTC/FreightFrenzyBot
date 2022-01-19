package org.firstinspires.ftc.teamcode.teleop;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.enums.DispenserPivot;
import org.firstinspires.ftc.teamcode.enums.SlideSetting;
import org.firstinspires.ftc.teamcode.enums.SlideState;
/*

    Mrs. Myers approves this code

*/

@Disabled
@TeleOp
public class TempTeleop extends OpMode {
    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    DcMotorEx slideExtend, slideRotate;
    Servo dispenseTilt, dispensePivot, dispenseGate;
    DcMotor intake;
    CRServo spinner1, spinner2;
    AnalogInput limitSwitch0 ,limitSwitch1;
    double spinPower = 1.0;
    SlideState curSlideState = SlideState.RETRACTED;
    SlideSetting curSlideSetting = SlideSetting.RETRACT;
    DispenserPivot curDispenserState = DispenserPivot.CENTER;
    final double SLIDE_RETRACT_POWER = -0.6, SLIDE_EXTEND_POWER = 0.8;
    int slideTargetPos = 0;
    final double tiltTicsFor90degrees = 2700.0; //Number of tics for 90 degrees of slide rotation (influences dispenser tilt)
    final int extendMinimum = 263; //Min slide extension tics before tilting dispenser or opening gate (must be above motor to prevent damage) (used for tilt and gate)
    final double tiltMinimum = 0.1012; //Min dispenser tilt servo position before pivoting servo (must not turn into the slides) (used for pivot)
    final double slideTiltScale = 0.85;
    //865 max rotation tics (if 0 is vertical)
    final double slideTiltMax = tiltTicsFor90degrees*slideTiltScale;
    final double driveSpeed = 0.6;
    final double rotateTicsDeltaToVertical = 1180.0;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        motorFrontLeft = hardwareMap.dcMotor.get("TLM10");
        motorBackLeft = hardwareMap.dcMotor.get("BLM11");
        motorFrontRight = hardwareMap.dcMotor.get("TRM12");
        motorBackRight = hardwareMap.dcMotor.get("BRM13");

        spinner1 = hardwareMap.crservo.get("RSS15");
        spinner2 = hardwareMap.crservo.get("BSS14");


        dispenseTilt = hardwareMap.get(Servo.class, "DRS11");
        dispensePivot = hardwareMap.get(Servo.class, "DPS12");
        dispenseGate = hardwareMap.get(Servo.class, "DGS13");

    }
    public void init_loop(){
        curSlideState = getSlideCurrentState();
        telemetry.addData("SLIDES ARE:", curSlideState);
    }
    @Override
    public void loop() {
        //TODO: coding at the pool
        intake();
        spinCarousel();
        drive();
        toggle();
        slideTilt();
        slideControl();
        moveSlides();
        dispenser();
        telemetry.addData("rot", slideRotate.getCurrentPosition());
        telemetry.addData("ext", slideExtend.getCurrentPosition());
//        telemetry.addData("TILT", (slideRotate.getCurrentPosition() / tiltTicsFor90degrees) * .35);
//        telemetry.addData("rotateTarget", dispenseTilt.getPosition());
//        telemetry.addData("slideExtend", slideExtend.getCurrentPosition());
//        telemetry.addData("slideRotate", slideRotate.getCurrentPosition());
//        telemetry.addData("dispenserTilt", dispenseTilt.getPosition());
//        telemetry.addData("dispenserPivot", dispensePivot.getPosition());
//        telemetry.addData("dispenseGate", dispenseGate.getPosition());
//        telemetry.addData("slidesRetracted", curSlideState);
//        telemetry.addData("targetPos", slideExtend.getTargetPosition());
//621 full extend
        //TODO: coding at the school
    }
    public void wait(double waitTime) {
        timer.reset();
        while (timer.time() < waitTime) {
        }
    }
    public void drive(){
        double scaleFactor = driveSpeed + gamepad1.left_trigger * (1-driveSpeed);
        double leftPower = -gamepad1.left_stick_y * scaleFactor;
        double rightPower = -gamepad1.right_stick_y * scaleFactor;
        motorFrontLeft.setPower(leftPower);
        motorBackLeft.setPower(leftPower);
        motorFrontRight.setPower(rightPower);
        motorBackRight.setPower(rightPower);
    }
    public void slideControl(){
        if(gamepad2.a){
            curSlideSetting = SlideSetting.EXTEND;
            slideTargetPos = 790;
        } else if(gamepad2.b) {
            curSlideSetting = SlideSetting.EXTEND;
            slideTargetPos = 1580;
        } else if(gamepad2.x){
            curSlideSetting = SlideSetting.RETRACT;
            slideTargetPos = 0;
        }
    }
    public SlideState getSlideCurrentState(){
        if(limitSwitch0.getVoltage() > 1 && limitSwitch1.getVoltage() < 1){
            return SlideState.EXTENDED;
        } else if(limitSwitch0.getVoltage() < 1 && limitSwitch1.getVoltage() > 1){
            return SlideState.RETRACTED;
        } else {
            telemetry.addLine("LIMIT SWITCH DISCONNECTED");
            return SlideState.ERROR;
        }
    }

    public void moveSlides() {
        curSlideState = getSlideCurrentState();
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
    public void slideTilt(){
        final double slideRotateScalar = 0.6;
        if(slideRotate.getCurrentPosition() >= slideTiltMax){
            slideRotate.setPower(-(Math.abs(gamepad2.left_stick_y * slideRotateScalar)));
        } else if(slideRotate.getCurrentPosition() <= 0){
            slideRotate.setPower(Math.abs(gamepad2.left_stick_y * slideRotateScalar));
        } else {
            slideRotate.setPower(-gamepad2.left_stick_y * slideRotateScalar);
        }
    }
    public void dispenser(){
        //Tilt:
        if(slideExtend.getCurrentPosition() > extendMinimum && curSlideSetting == SlideSetting.EXTEND && curSlideState == SlideState.EXTENDED) {
            dispenseTilt.setPosition(Math.max(0, ((slideRotate.getCurrentPosition()-rotateTicsDeltaToVertical) / tiltTicsFor90degrees) * .35));
        } else{
            dispenseTilt.setPosition(0.0);
        }
        //Gate:
        if(slideExtend.getCurrentPosition() > extendMinimum && curSlideSetting == SlideSetting.EXTEND && curSlideState == SlideState.EXTENDED) {
            if(gamepad2.left_bumper){
                dispenseGate.setPosition(0.45);
            } else {
                dispenseGate.setPosition(1.0);
            }
        } else {
            dispenseGate.setPosition(1.0);
        }
        //Pivot Setting:
        if(dispenseTilt.getPosition() > tiltMinimum && slideExtend.getCurrentPosition() > extendMinimum  && curSlideSetting == SlideSetting.EXTEND && curSlideState == SlideState.EXTENDED){
            if(gamepad2.dpad_up){
                curDispenserState = DispenserPivot.CENTER;
            } else if(gamepad2.dpad_left){
                curDispenserState = DispenserPivot.LEFT;
            } else if(gamepad2.dpad_right){
                curDispenserState = DispenserPivot.RIGHT;
            }
            pivotDispenser();
        } else {
            dispensePivot.setPosition(0.48);
            curDispenserState = DispenserPivot.CENTER;
        }
        if(gamepad2.left_trigger >= 1.0){
            slideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

    }
    public void pivotDispenser(){
        if(curDispenserState == DispenserPivot.CENTER){
            dispensePivot.setPosition(0.48);
        } else if(curDispenserState == DispenserPivot.LEFT){
            dispensePivot.setPosition(0.8);
        } else if(curDispenserState == DispenserPivot.RIGHT){
            dispensePivot.setPosition(0.15);
        }
    }
    public void intake() {
        if(gamepad2.right_trigger > 0 && curSlideState == SlideState.RETRACTED){
            intake.setPower(gamepad2.right_trigger);
        } else if(gamepad2.right_bumper){
            intake.setPower(-1.0);
        } else{
            intake.setPower(0.0);
        }
    }
    public void spinCarousel() {
        if(gamepad1.dpad_left) {
            spinner1.setPower(spinPower);
            spinner2.setPower(spinPower);
        } else if(gamepad1.dpad_right){
            spinner1.setPower(-spinPower);
            spinner2.setPower(-spinPower);
        } else{
            spinner1.setPower(0.0);
            spinner2.setPower(0.0);
        }
    }
    boolean oldA = false;
    boolean aToggled = false;
    boolean oldA2 = false;
    boolean a2Toggled = false;
    boolean oldY = false;
    boolean yToggled = false;
    boolean oldY2 = false;
    boolean y2Toggled = false;
    boolean oldB = false;
    boolean bToggled = false;
    boolean oldB2 = false;
    boolean b2Toggled = false;
    boolean oldRB2 = false;
    boolean rb2Toggled = false;
    boolean oldLB2 = false;
    boolean lb2Toggled = false;
    boolean oldDpadDown2 = false;
    boolean dpadDown2Toggled = false;
    boolean oldDpadRight2 = false;
    boolean dpadRight2Toggled = false;
    boolean oldDpadLeft2 = false;
    boolean dpadLeft2Toggled = false;
    boolean oldDpadUp2 = false;
    boolean dpadUp2Toggled = false;
    void toggle(){
        if(!oldA && gamepad1.a){aToggled=!aToggled;}
        oldA = gamepad1.a;
        if(!oldA2 && gamepad2.a){a2Toggled=!a2Toggled;}
        oldA2 = gamepad2.a;
        if(!oldY && gamepad1.y){yToggled=!yToggled;}
        oldY = gamepad1.y;
        if(!oldY2 && gamepad2.y){y2Toggled=!y2Toggled;}
        oldY2 = gamepad2.y;
        if(!oldB && gamepad1.b){bToggled=!bToggled;}
        oldB = gamepad1.b;
        if(!oldB2 && gamepad2.b){b2Toggled=!b2Toggled;}
        oldB2 = gamepad2.b;
        if(!oldRB2 && gamepad2.right_bumper){rb2Toggled=!rb2Toggled;}
        oldRB2 = gamepad2.right_bumper;
        if(!oldLB2 && gamepad2.left_bumper){lb2Toggled=!lb2Toggled;}
        oldLB2 = gamepad2.left_bumper;
        if(!oldDpadDown2 && gamepad2.dpad_down){dpadDown2Toggled =!dpadDown2Toggled;}
        oldDpadDown2 = gamepad2.dpad_down;
        if(!oldDpadRight2 && gamepad2.dpad_right){dpadRight2Toggled =!dpadRight2Toggled;}
        oldDpadRight2 = gamepad2.dpad_right;
        if(!oldDpadLeft2 && gamepad2.dpad_left){dpadLeft2Toggled =!dpadLeft2Toggled;}
        oldDpadLeft2 = gamepad2.dpad_left;
        if(!oldDpadUp2 && gamepad2.dpad_up){dpadUp2Toggled =!dpadUp2Toggled;}
        oldDpadUp2 = gamepad2.dpad_up;
    }
}
//code to survive
/*
Coding by Jonas Ho



 */