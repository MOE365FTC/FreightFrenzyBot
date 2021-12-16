package org.firstinspires.ftc.teamcode.teleop;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class TempTeleop extends OpMode {
    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    DcMotorEx arm;
    Servo servoArm, outtake, grabber;
    DcMotor intake;
    CRServo spinner;
    double armPower = 0.3;
    int armPos = 0;
    double spinPower = 0.5;

    @Override
    public void init() {
        motorFrontLeft = hardwareMap.dcMotor.get("TLM10");
        motorBackLeft = hardwareMap.dcMotor.get("BLM11");
        motorFrontRight = hardwareMap.dcMotor.get("TRM12");
        motorBackRight = hardwareMap.dcMotor.get("BRM13");
        arm = hardwareMap.get(DcMotorEx.class, "arm20"); //arm
//        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        arm.setTargetPosition(0);
//        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        servoArm = hardwareMap.get(Servo.class, "servoArm12");
//        grabber = hardwareMap.servo.get("grabber");
        outtake = hardwareMap.servo.get("out11");
        spinner = hardwareMap.crservo.get("blueSpinner");
        intake = hardwareMap.dcMotor.get("intake23");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    @Override
    public void loop() {
        intake();
        spinCarousel();
        drive();
        toggle();
        arm();
        servoArm();
        telemetry.addData("armPos", arm.getCurrentPosition());
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
            intake.setPower(0.7);
        } else{
            intake.setPower(0.0);
        }
    }
    public void servoArm(){
        if(y2Toggled){
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
        } else{
            spinner.setPower(0.0);
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
    }
}