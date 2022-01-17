//package org.firstinspires.ftc.teamcode.functions;
//
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.teamcode.functions.enums.slideSetting;
//import org.firstinspires.ftc.teamcode.functions.enums.slideState;
//
//public class Slides {
//    public slideState curSlideState = slideState.RETRACTED;
//    public slideSetting curSlideSetting = slideSetting.RETRACT;
//    public int slideTargetPos = 0;
//    AnalogInput limitSwitch0 ,limitSwitch1;
//    DcMotorEx slideExtend, slideRotate;
//    public final double SLIDE_RETRACT_POWER = -0.6, SLIDE_EXTEND_POWER = 0.8;
//    public final double tiltTicsFor90degrees = 1100.0; //Number of tics for 90 degrees of slide rotation (influences dispenser tilt)
//    public final double slideTiltScale = 0.75;
//    public final double slideTiltMax = tiltTicsFor90degrees*slideTiltScale;
//    public final double slideTiltMin = 377; //amount of tics for slides to be vertical relative to tilted starting position
//
//    public void slideControl(){
//        if(gamepad2.a){
//            curSlideSetting = slideSetting.EXTEND;
//            slideTargetPos = 790;
//        } else if(gamepad2.b) {
//            curSlideSetting = slideSetting.EXTEND;
//            slideTargetPos = 1580;
//        } else if(gamepad2.x){
//            curSlideSetting = slideSetting.RETRACT;
//            slideTargetPos = 0;
//        }
//    }
//    public slideState getSlideCurrentState(){
//        if(limitSwitch0.getVoltage() > 1 && limitSwitch1.getVoltage() < 1){
//            return slideState.EXTENDED;
//        } else if(limitSwitch0.getVoltage() < 1 && limitSwitch1.getVoltage() > 1){
//            return slideState.RETRACTED;
//        } else {
//            telemetry.addLine("LIMIT SWITCH DISCONNECTED");
//            return slideState.ERROR;
//        }
//    }
//
//    public void moveSlides() {
//        curSlideState = getSlideCurrentState();
//        if(curSlideState == slideState.RETRACTED && slideExtend.getCurrentPosition() != 0 && curSlideSetting != slideSetting.MANUAL_OVERRIDE){
//            slideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        if(curSlideSetting == slideSetting.RETRACT && curSlideState != slideState.RETRACTED && curSlideSetting != slideSetting.MANUAL_OVERRIDE) {
//            slideExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            slideExtend.setPower(SLIDE_RETRACT_POWER);
//        } else if(curSlideSetting == slideSetting.EXTEND){
//            slideExtend.setTargetPosition(slideTargetPos);
//            slideExtend.setPower(SLIDE_EXTEND_POWER);
//            slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        } else if(curSlideSetting == slideSetting.MANUAL_OVERRIDE){
//            slideExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//            slideExtend.setPower(gamepad2.right_stick_y);
//        } else{
//            slideExtend.setPower(0);
//        }
//    }
//    public void slideTilt(){
//        final double slideRotateScalar = 0.3;
//        if(slideRotate.getCurrentPosition() >= slideTiltMax){
//            slideRotate.setPower(-(Math.abs(gamepad2.left_stick_y * slideRotateScalar)));
//        } else if(slideRotate.getCurrentPosition() <= 0){
//            slideRotate.setPower(Math.abs(gamepad2.left_stick_y * slideRotateScalar));
//        } else {
//            slideRotate.setPower(-gamepad2.left_stick_y * slideRotateScalar);
//        }
//    }
//}
