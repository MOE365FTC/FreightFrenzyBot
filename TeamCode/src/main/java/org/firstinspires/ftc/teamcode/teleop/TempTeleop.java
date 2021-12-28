package org.firstinspires.ftc.teamcode.teleop;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

//x2 intake
//y2 toggle servo arm
//b2 outtake
//rightsticky2 arm
//a2 toggle duckspinner
@TeleOp
public class TempTeleop extends OpMode {
    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    DcMotorEx arm, slideExtend, slideRotate;
    Servo servoArm, outtake, grabber;
    DcMotor intake;
    CRServo spinner;
    TouchSensor limitSwitch;
    double armPower = 0.3;
    int armPos = 0;
    double spinPower = 1.0;
    int STORED_EXTEND = 0, LOW_EXTEND = 250, MID_EXTEND = 250, HIGH_EXTEND = 250;
    int STORED_ROTATE = 0, LOW_ROTATE = 100, MID_ROTATE = 100, HIGH_ROTATE = 100;
    int SLIDE_MULTIPLIER = 1;
    @Override
    public void init() {
        motorFrontLeft = hardwareMap.dcMotor.get("TLM10");
        motorBackLeft = hardwareMap.dcMotor.get("BLM11");
        motorFrontRight = hardwareMap.dcMotor.get("TRM12");
        motorBackRight = hardwareMap.dcMotor.get("BRM13");

        arm = hardwareMap.get(DcMotorEx.class, "arm20"); //arm
        servoArm = hardwareMap.get(Servo.class, "servoArm12");
        outtake = hardwareMap.servo.get("out11");
        spinner = hardwareMap.crservo.get("blueSpinner");
        intake = hardwareMap.dcMotor.get("intake23");
        limitSwitch = hardwareMap.touchSensor.get("limit");

        slideExtend = hardwareMap.get(DcMotorEx.class, "SEM");
        slideRotate = hardwareMap.get(DcMotorEx.class, "SRM");

        slideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideExtend.setTargetPosition(0);
        slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRotate.setTargetPosition(0);
        slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop() {
        intake();
//        spinCarousel();
        drive();
        toggle();
//        arm();
        servoArm();
        slides();
        telemetry.addData("slideExtend", slideExtend.getCurrentPosition());
        slideExtend.setPower(0.5);
        if(limitSwitch.isPressed()){
            slideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void drive(){
        double scaleFactor = 0.8;
        double leftPower = -gamepad1.left_stick_y * scaleFactor;
        double rightPower = -gamepad1.right_stick_y * scaleFactor;
        motorFrontLeft.setPower(leftPower);
        motorBackLeft.setPower(leftPower);
        motorFrontRight.setPower(rightPower);
        motorBackRight.setPower(rightPower);
    }
    public void intake() {
        if(gamepad2.x){
            intake.setPower(1.0);
        } else if(gamepad2.y){
            intake.setPower(1.0);
        } else{
            intake.setPower(0.0);
        }
    }
    public void servoArm(){
        if(rb2Toggled){
            servoArm.setPosition(0.8);
        } else{
            servoArm.setPosition(0.2);
        }
    }
    public void arm(){
//        arm.setPower(armPower);
//        arm.setTargetPosition(armPos);
//        if(armPos >= 0 && armPos<=200){
//            if(gamepad1.dpad_up){
//                armPos += 2;
//                arm.setTargetPosition(armPos);
//            } else if(gamepad1.dpad_down){
//                armPos-=2;
//                arm.setTargetPosition(armPos);
//            }
//        } else if(armPos < 0){
//            armPos = 0;
//            arm.setTargetPosition(armPos);
//        } else {
//            armPos = 200;
//            arm.setTargetPosition(armPos);
//        }
////        if(Toggle.yToggled){
////            grabber.setPosition(1.0);
////        } else{
////            grabber.setPosition(0.0);
//        }
        if(b2Toggled){
            outtake.setPosition(0.4);
        } else{
            outtake.setPosition(0.0);
        }

            arm.setPower(gamepad2.right_stick_y * 0.5);
    }
    public void spinCarousel() {
//        if (aToggled) {
//            spinner.setPower(spinPower);
//        } else {
//            spinner.setPower(0.0);
//        }
        if(a2Toggled){
            spinner.setPower(spinPower);
        } else if(aToggled){
            spinner.setPower(-spinPower);
        } else{
            spinner.setPower(0.0);
        }
    }
    public void slides() {
        if(a2Toggled){
            int targetExtend = slideExtend.getTargetPosition();
            int targetRotate = slideRotate.getTargetPosition();
            targetExtend+=(int)gamepad2.right_stick_y * SLIDE_MULTIPLIER;
            targetRotate+=(int)gamepad2.left_stick_y * SLIDE_MULTIPLIER;
            slideRotate.setTargetPosition(targetExtend);
            slideExtend.setTargetPosition(targetRotate);
        } else{
            if(dpadDown2Toggled){
                slideExtend.setTargetPosition(LOW_EXTEND);
                dpadUp2Toggled = false;
                dpadRight2Toggled = false;
            } else if(dpadRight2Toggled){
                slideExtend.setTargetPosition(MID_EXTEND);
                dpadUp2Toggled = false;
                dpadDown2Toggled = false;
            } else if(dpadUp2Toggled){
                slideExtend.setTargetPosition(HIGH_EXTEND);
                dpadDown2Toggled = false;
                dpadRight2Toggled = false;
            }
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
        if(!oldDpadUp2 && gamepad2.dpad_up){dpadUp2Toggled =!dpadUp2Toggled;}
        oldDpadUp2 = gamepad2.dpad_up;
    }
}