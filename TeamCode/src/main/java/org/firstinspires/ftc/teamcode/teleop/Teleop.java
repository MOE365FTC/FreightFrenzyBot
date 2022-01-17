//package org.firstinspires.ftc.teamcode.teleop;
//
////import com.acmerobotics.dashboard.FtcDashboard;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.AnalogInput;
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.functions.*;
//import org.firstinspires.ftc.teamcode.functions.enums.*;
///*
//
//    Mrs. Myers approves this code
//
//*/
//
//enum dispenserPivot{
//    LEFT,
//    CENTER,
//    RIGHT
//}
//@TeleOp
//public class Teleop extends OpMode {
//    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
//    DcMotorEx slideExtend, slideRotate;
//    Servo dispenseTilt, dispensePivot, dispenseGate;
//    DcMotor intake;
//    CRServo spinner;
//    AnalogInput limitSwitch0 ,limitSwitch1;
//    double spinPower = 1.0;
//    dispenserPivot curDispenserState = dispenserPivot.CENTER;
//    final int extendMinimum = 100; //Min slide extension tics before tilting dispenser or opening gate (must be above motor to prevent damage) (used for tilt and gate)
//    final double tiltMinimum = 0.1012; //Min dispenser tilt servo position before pivoting servo (must not turn into the slides) (used for pivot)
//    final double driveSpeed = 0.4;
//    final double rotateTicsDeltaToVertical = 405.0;
//    ElapsedTime timer = new ElapsedTime();
//
//    Slides slides = new Slides();
//    @Override
//    public void init() {
//        motorFrontLeft = hardwareMap.dcMotor.get("TLM10");
//        motorBackLeft = hardwareMap.dcMotor.get("BLM11");
//        motorFrontRight = hardwareMap.dcMotor.get("TRM12");
//        motorBackRight = hardwareMap.dcMotor.get("BRM13");
//
//        spinner = hardwareMap.crservo.get("BSS15");
//        intake = hardwareMap.dcMotor.get("INM23");
//        limitSwitch0 = hardwareMap.get(AnalogInput.class, "LSD20");
//        limitSwitch1 = hardwareMap.get(AnalogInput.class, "LSD21");
//
//        slideExtend = hardwareMap.get(DcMotorEx.class, "SEM20");
//        slideRotate = hardwareMap.get(DcMotorEx.class, "SRM21");
//
//        dispenseTilt = hardwareMap.get(Servo.class, "DRS11");
//        dispensePivot = hardwareMap.get(Servo.class, "DPS12");
//        dispenseGate = hardwareMap.get(Servo.class, "DGS13");
//
//        slideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideExtend.setTargetPosition(0);
//        slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        slideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
////        slideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
////        slideRotate.setTargetPosition(0);
////        slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        slideExtend.setDirection(DcMotor.Direction.REVERSE);
//        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
//        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
//        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        slideRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//    public void init_loop(){
//        slides.curSlideState = slides.getSlideCurrentState();
//        telemetry.addData("SLIDES ARE:", slides.curSlideState);
//    }
//    @Override
//    public void loop() {
//        //TODO: coding at the pool
//        intake();
//        spinCarousel();
//        drive();
//        toggle();
//        slides.slideTilt();
//        slides.slideControl();
//        slides.moveSlides();
//        dispenser();
//        telemetry.addData("TILT", (slideRotate.getCurrentPosition() / slides.tiltTicsFor90degrees) * .35);
//        telemetry.addData("rotateTarget", dispenseTilt.getPosition());
//        telemetry.addData("slideExtend", slideExtend.getCurrentPosition());
//        telemetry.addData("slideRotate", slideRotate.getCurrentPosition());
//        telemetry.addData("dispenserTilt", dispenseTilt.getPosition());
//        telemetry.addData("dispenserPivot", dispensePivot.getPosition());
//        telemetry.addData("dispenseGate", dispenseGate.getPosition());
//        telemetry.addData("slidesRetracted", slides.curSlideState);
//        telemetry.addData("targetPos", slideExtend.getTargetPosition());
////621 full extend
//        //TODO: coding at the school
//    }
//    public void wait(double waitTime) {
//        timer.reset();
//        while (timer.time() < waitTime) {
//        }
//    }
//    public void drive(){
//        double scaleFactor = driveSpeed + gamepad1.left_trigger * (1-driveSpeed);
//        double leftPower = -gamepad1.left_stick_y * scaleFactor;
//        double rightPower = -gamepad1.right_stick_y * scaleFactor;
//        motorFrontLeft.setPower(leftPower);
//        motorBackLeft.setPower(leftPower);
//        motorFrontRight.setPower(rightPower);
//        motorBackRight.setPower(rightPower);
//    }
//
//    public void dispenser(){
//        //Tilt:
//        if(slideExtend.getCurrentPosition() > extendMinimum && slides.curSlideSetting == slideSetting.EXTEND && slides.curSlideState == slideState.EXTENDED) {
//            dispenseTilt.setPosition(Math.max(0, ((slideRotate.getCurrentPosition()-rotateTicsDeltaToVertical) / slides.tiltTicsFor90degrees) * .35));
//        } else{
//            dispenseTilt.setPosition(0.0);
//        }
//        //Gate:
//        //TODO: Make resting gate position slightly forward so balls don't get stuck
//        if(slideExtend.getCurrentPosition() > extendMinimum && slides.curSlideSetting == slideSetting.EXTEND && slides.curSlideState == slideState.EXTENDED) {
//            if(gamepad2.left_bumper){
//                dispenseGate.setPosition(0.45);
//            } else {
//                dispenseGate.setPosition(1.0);
//            }
//        } else {
//            dispenseGate.setPosition(1.0);
//        }
//        //Pivot Setting:
//        if(dispenseTilt.getPosition() > tiltMinimum && slideExtend.getCurrentPosition() > extendMinimum  && slides.curSlideSetting == slideSetting.EXTEND && slides.curSlideState == slideState.EXTENDED){
//            if(gamepad2.dpad_up){
//                curDispenserState = dispenserPivot.CENTER;
//            } else if(gamepad2.dpad_left){
//                curDispenserState = dispenserPivot.LEFT;
//            } else if(gamepad2.dpad_right){
//                curDispenserState = dispenserPivot.RIGHT;
//            }
//            pivotDispenser();
//        } else {
//            dispensePivot.setPosition(0.48);
//            curDispenserState = dispenserPivot.CENTER;
//        }
//        if(gamepad2.left_trigger >= 1.0){
//            slideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            slideRotate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//
//    }
//    public void pivotDispenser(){
//        if(curDispenserState == dispenserPivot.CENTER){
//            dispensePivot.setPosition(0.48);
//        } else if(curDispenserState == dispenserPivot.LEFT){
//            dispensePivot.setPosition(0.8);
//        } else if(curDispenserState == dispenserPivot.RIGHT){
//            dispensePivot.setPosition(0.15);
//        }
//    }
//    public void intake() {
//        if(gamepad2.right_trigger > 0 && slides.curSlideState == slideState.RETRACTED){
//            intake.setPower(gamepad2.right_trigger);
//        } else if(gamepad2.right_bumper){
//            intake.setPower(-1.0);
//        } else{
//            intake.setPower(0.0);
//        }
//    }
//    public void spinCarousel() {
//        if(gamepad1.dpad_left) {
//            spinner.setPower(spinPower);
//        } else if(gamepad1.dpad_right){
//            spinner.setPower(-spinPower);
//        } else{
//            spinner.setPower(0.0);
//        }
//    }
//    boolean oldA = false;
//    boolean aToggled = false;
//    boolean oldA2 = false;
//    boolean a2Toggled = false;
//    boolean oldY = false;
//    boolean yToggled = false;
//    boolean oldY2 = false;
//    boolean y2Toggled = false;
//    boolean oldB = false;
//    boolean bToggled = false;
//    boolean oldB2 = false;
//    boolean b2Toggled = false;
//    boolean oldRB2 = false;
//    boolean rb2Toggled = false;
//    boolean oldLB2 = false;
//    boolean lb2Toggled = false;
//    boolean oldDpadDown2 = false;
//    boolean dpadDown2Toggled = false;
//    boolean oldDpadRight2 = false;
//    boolean dpadRight2Toggled = false;
//    boolean oldDpadLeft2 = false;
//    boolean dpadLeft2Toggled = false;
//    boolean oldDpadUp2 = false;
//    boolean dpadUp2Toggled = false;
//    void toggle(){
//        if(!oldA && gamepad1.a){aToggled=!aToggled;}
//        oldA = gamepad1.a;
//        if(!oldA2 && gamepad2.a){a2Toggled=!a2Toggled;}
//        oldA2 = gamepad2.a;
//        if(!oldY && gamepad1.y){yToggled=!yToggled;}
//        oldY = gamepad1.y;
//        if(!oldY2 && gamepad2.y){y2Toggled=!y2Toggled;}
//        oldY2 = gamepad2.y;
//        if(!oldB && gamepad1.b){bToggled=!bToggled;}
//        oldB = gamepad1.b;
//        if(!oldB2 && gamepad2.b){b2Toggled=!b2Toggled;}
//        oldB2 = gamepad2.b;
//        if(!oldRB2 && gamepad2.right_bumper){rb2Toggled=!rb2Toggled;}
//        oldRB2 = gamepad2.right_bumper;
//        if(!oldLB2 && gamepad2.left_bumper){lb2Toggled=!lb2Toggled;}
//        oldLB2 = gamepad2.left_bumper;
//        if(!oldDpadDown2 && gamepad2.dpad_down){dpadDown2Toggled =!dpadDown2Toggled;}
//        oldDpadDown2 = gamepad2.dpad_down;
//        if(!oldDpadRight2 && gamepad2.dpad_right){dpadRight2Toggled =!dpadRight2Toggled;}
//        oldDpadRight2 = gamepad2.dpad_right;
//        if(!oldDpadLeft2 && gamepad2.dpad_left){dpadLeft2Toggled =!dpadLeft2Toggled;}
//        oldDpadLeft2 = gamepad2.dpad_left;
//        if(!oldDpadUp2 && gamepad2.dpad_up){dpadUp2Toggled =!dpadUp2Toggled;}
//        oldDpadUp2 = gamepad2.dpad_up;
//    }
//}
////code to survive
///*
//Coding by Jonas Ho
//
//
//
// */