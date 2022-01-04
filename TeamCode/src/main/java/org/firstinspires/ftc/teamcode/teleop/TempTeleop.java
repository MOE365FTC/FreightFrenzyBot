package org.firstinspires.ftc.teamcode.teleop;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



//Mrs. Myers approves this code

enum slideState{
    EXTENDED,
    RETRACTED,
    ERROR
}

enum slideSetting{
    EXTEND,
    RETRACT
}
@TeleOp
public class TempTeleop extends OpMode {
    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    DcMotorEx slideExtend, slideRotate;
    Servo dispenseRotate, dispensePivot, dispenseGate;
    DcMotor intake;
    CRServo spinner;
    AnalogInput limitSwitch0 ,limitSwitch1;
    double spinPower = 1.0, dispenseUp = 1.0, dispenseDown = 0.0, dispenseOpen = 1.0, dispenseClose = 0.0;
    int STORED_EXTEND = 0, LOW_EXTEND = 250, MID_EXTEND = 250, HIGH_EXTEND = 250;
    int STORED_ROTATE = 0, LOW_ROTATE = 100, MID_ROTATE = 100, HIGH_ROTATE = 100;
    int SLIDE_MULTIPLIER = 1;
    slideState curSlideState = slideState.RETRACTED;
    slideSetting curSlideSetting = slideSetting.RETRACT;
    double SLIDE_RETRACT_POWER = -0.6;
    double SLIDE_EXTEND_POWER = 0.8;
    int slideTargetPos = 0;
    ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        motorFrontLeft = hardwareMap.dcMotor.get("TLM10");
        motorBackLeft = hardwareMap.dcMotor.get("BLM11");
        motorFrontRight = hardwareMap.dcMotor.get("TRM12");
        motorBackRight = hardwareMap.dcMotor.get("BRM13");

        spinner = hardwareMap.crservo.get("BSS15");
        intake = hardwareMap.dcMotor.get("INM23");
        limitSwitch0 = hardwareMap.get(AnalogInput.class, "LSD20");
        limitSwitch1 = hardwareMap.get(AnalogInput.class, "LSD21");
//        limitSwitch0.setMode(DigitalChannel.Mode.INPUT);
//        limitSwitch1.setMode(DigitalChannel.Mode.INPUT);
        slideExtend = hardwareMap.get(DcMotorEx.class, "SEM20");
        slideRotate = hardwareMap.get(DcMotorEx.class, "SRM21");

        dispenseRotate = hardwareMap.get(Servo.class, "DRS11");
        dispensePivot = hardwareMap.get(Servo.class, "DPS12");
        dispenseGate = hardwareMap.get(Servo.class, "DGS13");

        slideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideExtend.setTargetPosition(0);
        slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

//        slideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        slideRotate.setTargetPosition(0);
//        slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideExtend.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void init_loop(){
        curSlideState = getSlideCurrentState();
        telemetry.addData("SLIDES ARE:", curSlideState);
    }
    @Override
    public void loop() {
        //coding at the pool
        intake();
//        spinCarousel();
        drive();
//        toggle();
//        servoArm();
//        slides();
        if(gamepad2.a){
            curSlideSetting = slideSetting.EXTEND;
            slideTargetPos = 300;
        } else if(gamepad2.b){
            curSlideSetting = slideSetting.EXTEND;
            slideTargetPos = 600;
        } else if(gamepad2.x){
            curSlideSetting = slideSetting.RETRACT;
            slideTargetPos = 0;
        }
        moveSlides();
        telemetry.addData("slideExtend", slideExtend.getCurrentPosition());
        telemetry.addData("slideRotate", slideRotate.getCurrentPosition());
        telemetry.addData("dispenserRotate", dispenseRotate.getPosition());
        telemetry.addData("dispenserPivot", dispensePivot.getPosition());
        telemetry.addData("dispenseGate", dispenseGate.getPosition());
        telemetry.addData("slidesRetracted", curSlideState);
        telemetry.addData("targetPos", slideExtend.getTargetPosition());
//        telemetry.addData("0", limitSwitch0.getVoltage());
//        telemetry.addData("1", limitSwitch1.getVoltage());
//        slideExtend.setPower(0.5);
//-621 full extend
        tilt();
    }
    public void wait(double waitTime) {
        timer.reset();
        while (timer.time() < waitTime) {
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
    public slideState getSlideCurrentState(){
        if(limitSwitch0.getVoltage() > 1 && limitSwitch1.getVoltage() < 1){
            return slideState.EXTENDED;
        } else if(limitSwitch0.getVoltage() < 1 && limitSwitch1.getVoltage() > 1){
            return slideState.RETRACTED;
        } else {
            telemetry.addLine("LIMIT SWITCH DISCONNECTED");
            return slideState.ERROR;
        }
    }

    public void moveSlides(){
        curSlideState = getSlideCurrentState();
        if(curSlideState == slideState.RETRACTED && slideExtend.getCurrentPosition() != 0){
            slideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if(curSlideSetting == slideSetting.RETRACT && curSlideState != slideState.RETRACTED) {
            slideExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slideExtend.setPower(SLIDE_RETRACT_POWER);
        } else if(curSlideSetting == slideSetting.EXTEND){
            slideExtend.setTargetPosition(slideTargetPos);
            slideExtend.setPower(SLIDE_EXTEND_POWER);
            slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        } else{
            slideExtend.setPower(0);
        }
    }
    public void tilt(){
        slideRotate.setPower(gamepad2.left_stick_y);
    }
    public void intake() {
        if(gamepad2.right_bumper && curSlideState == slideState.RETRACTED){
            intake.setPower(1.0);
        } else if(gamepad2.left_bumper){
            intake.setPower(1.0);
        } else{
            intake.setPower(0.0);
        }
    }

    public void spinCarousel() {
        if(a2Toggled) {
            spinner.setPower(spinPower);
        } //TODO: else if something set to -spinPower (for other side carousel)
        else{
            spinner.setPower(0.0);
        }
    }
    public void slides() {
        int targetExtend = slideExtend.getTargetPosition();
        int targetRotate = slideRotate.getTargetPosition();
        targetExtend+=(int)gamepad2.right_stick_y * SLIDE_MULTIPLIER;
        targetRotate+=(int)gamepad2.left_stick_y * SLIDE_MULTIPLIER;
        slideRotate.setTargetPosition(targetExtend);
        slideExtend.setTargetPosition(targetRotate);
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
        } else {
            dispenseRotate.setPosition(dispenseDown);
            wait(1.0);
            slideExtend.setTargetPosition(STORED_EXTEND);
            dpadDown2Toggled = false;
            dpadRight2Toggled = false;
            dpadUp2Toggled = false;
        }
        if(slideExtend.getTargetPosition() > 100){ //notInsideBot
            if(aToggled){
                dispenseRotate.setPosition(dispenseUp);
                wait(0.5);
                dispenseGate.setPosition(dispenseOpen);
            } else {
                dispenseRotate.setPosition(dispenseDown);
            }
        } else {
            if(aToggled){aToggled = false;}
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