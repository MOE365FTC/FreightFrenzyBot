package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

enum slideState{
    EXTENDED,
    RETRACTED,
    ERROR
}

enum slideSetting{
    EXTEND,
    RETRACT,
    MANUAL_OVERRIDE
}

enum Direction{
    LEFT,
    RIGHT
}

enum tse{
        TOP,
        MID,
        BOT
        }
public class BlueTopAuton extends LinearOpMode {
    BNO055IMU imu;
    double HEADING_OFFSET = 90; // starting heading of the robot, change this for every auton starting pose

    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    DcMotorEx slideExtend, slideRotate;
    Servo dispenseTilt, dispensePivot, dispenseGate;
    DcMotor intake;
    CRServo spinner;
    AnalogInput limitSwitch0 ,limitSwitch1;
    double spinPower = 1.0;
    final int SLIDE_MULTIPLIER = 1;
    slideState curSlideState = slideState.RETRACTED;
    slideSetting curSlideSetting = slideSetting.RETRACT;
    final double SLIDE_RETRACT_POWER = -0.3, SLIDE_EXTEND_POWER = 0.8;
    int slideTargetPos = 0;
    final int tiltTicsFor90degrees = 1453; //Number of tics for 90 degrees of slide rotation (influences dispenser tilt)
    final int extendMinimum = 100; //Min slide extension tics before tilting dispenser or opening gate (must be above motor to prevent damage) (used for tilt and gate)
    final double tiltMinimum = 0.35/2; //Min dispenser tilt servo position before pivoting servo (must not turn into the slides) (used for pivot)

    final double secondsPerInch = 0.2; //# of seconds of full motor power to move 1 inch (~12V Battery)
    ElapsedTime timer = new ElapsedTime();

    public void wait(double waitTime) {
        timer.reset();
        while(timer.time() < waitTime && opModeIsActive()) {
        }
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
//    public void moveSlides(){
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
    public void dispenser(){
        //Tilt:
        if(slideExtend.getCurrentPosition() > extendMinimum && curSlideSetting == slideSetting.EXTEND && curSlideState == slideState.EXTENDED) {
            dispenseTilt.setPosition((slideRotate.getCurrentPosition() / tiltTicsFor90degrees) * .35);
        } else{
            dispenseTilt.setPosition(0.0);
        }
        //Gate:
        if(slideExtend.getCurrentPosition() > extendMinimum && curSlideSetting == slideSetting.EXTEND && curSlideState == slideState.EXTENDED) {
            if(gamepad2.left_bumper){
                dispenseGate.setPosition(0.45);
            } else {
                dispenseGate.setPosition(1.0);
            }
        } else {
            dispenseGate.setPosition(1.0);
        }
        //Pivot:
        if(dispenseTilt.getPosition() > tiltMinimum && slideExtend.getCurrentPosition() > extendMinimum  &&  curSlideSetting == slideSetting.EXTEND && curSlideState == slideState.EXTENDED){
            if(dpadLeft2Toggled){
                dispensePivot.setPosition(0.8);
                dpadRight2Toggled = false;
                dpadUp2Toggled = false;
            } else if(dpadUp2Toggled){
                dispensePivot.setPosition(0.0);
                dpadRight2Toggled = false;
                dpadLeft2Toggled = false;
            } else if(dpadRight2Toggled){
                dispensePivot.setPosition(0.15);
                dpadLeft2Toggled = false;
                dpadUp2Toggled = false;
            } else {
                dispensePivot.setPosition(0.0);
            }
        } else {
            dispensePivot.setPosition(0.0);
        }
    }
    public void intake() {
        if(gamepad2.right_trigger > 0 && curSlideState == slideState.RETRACTED){
            intake.setPower(gamepad2.right_trigger);
        } else if(gamepad2.right_bumper){
            intake.setPower(-1.0);
        } else{
            intake.setPower(0.0);
        }
    }
    public void spinCarousel() {
        if(gamepad1.dpad_left) {
            spinner.setPower(spinPower);
        } else if(gamepad1.dpad_right){
            spinner.setPower(spinPower);
        } else{
            spinner.setPower(0.0);
        }
    }
    public void manualDrive(double leftPower, double rightPower, double time){
        motorBackLeft.setPower(leftPower);
        motorFrontLeft.setPower(leftPower);
        motorBackRight.setPower(rightPower);
        motorFrontRight.setPower(rightPower);
        wait(time);
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
    }
    public void forward(double distance){
        manualDrive(1.0,1.0, distance*secondsPerInch);
    }
    public void back(double distance){
        manualDrive(1.0,1.0, distance*secondsPerInch);
    }
    public void turnToHeading(double targetHeading, Direction dir, double minPower, double maxPower, double tolerance) {
        //targetHeading: final heading to face
        //dir: Direction to turn
        //minPower: minimum power to turn the robot at
        //maxPower: max power to turn the robot at (if min == max, constant turning speed)
        //tolerance: tolerance (in degrees) to stop the turn at
        double turnScaling = 0.1; //TODO: coefficient to modify the error by, trial and error here
        double imuHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
        double curHeading = imuHeading + HEADING_OFFSET; // this is our 'adjusted' current heading
        int multiplier = dir == Direction.RIGHT ? 1 : -1; // multiply powers by 1 if RIGHT, -1 if LEFT to get the turn direction
        double error = Math.abs(curHeading - targetHeading);
        double turnPower = clamp(error * turnScaling, minPower, maxPower); // get a power to turn at, won't ever go below min or over max power
        while (error > tolerance && opModeIsActive()) { // ALWAYS put an opModeIsActive() check in all FTC while loops!!!
            imuHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            curHeading = imuHeading + HEADING_OFFSET;
            error = Math.abs(curHeading - targetHeading);
            turnPower = clamp(error * turnScaling, minPower, maxPower);
            motorFrontRight.setPower(multiplier * turnPower);
            motorBackRight.setPower(multiplier * turnPower);
            motorFrontLeft.setPower(-1 * multiplier * turnPower);
            motorBackLeft.setPower(-1 * multiplier * turnPower);
//            sleep(10); // sleep for 10 milliseconds, we can tune this number too.
            telemetry.addData("error", error);
            telemetry.addData("1", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("2", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
            telemetry.addData("3", imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
            telemetry.update();
        }
    }
    public void moveArm(tse curTse){
        switch(curTse){
            case TOP:
                slideExtend.setTargetPosition(600);
                slideExtend.setPower(0.8);
                slideRotate.setTargetPosition(175);
                slideRotate.setPower(0.5);
                gamepad1.left_bumper = true;
                dispenser();
                wait(1.0);
                gamepad1.left_bumper = false;
                break;
            case MID:
                slideExtend.setTargetPosition(500);
                slideExtend.setPower(0.8);
                slideRotate.setTargetPosition(250);
                slideRotate.setPower(0.5);
                gamepad1.left_bumper = true;
                dispenser();
                wait(1.0);
                gamepad1.left_bumper = false;
                break;
            case BOT:
                slideExtend.setTargetPosition(400);
                slideExtend.setPower(0.8);
                slideRotate.setTargetPosition(100);
                slideRotate.setPower(0.5);
                gamepad1.left_bumper = true;
                dispenser();
                wait(1.0);
                gamepad1.left_bumper = false;
                break;
        }
    }
    public void runOpMode() {
        motorFrontLeft = hardwareMap.dcMotor.get("TLM10");
        motorBackLeft = hardwareMap.dcMotor.get("BLM11");
        motorFrontRight = hardwareMap.dcMotor.get("TRM12");
        motorBackRight = hardwareMap.dcMotor.get("BRM13");

        spinner = hardwareMap.crservo.get("BSS15");
        intake = hardwareMap.dcMotor.get("INM23");
        limitSwitch0 = hardwareMap.get(AnalogInput.class, "LSD20");
        limitSwitch1 = hardwareMap.get(AnalogInput.class, "LSD21");

        slideExtend = hardwareMap.get(DcMotorEx.class, "SEM20");
        slideRotate = hardwareMap.get(DcMotorEx.class, "SRM21");

        dispenseTilt = hardwareMap.get(Servo.class, "DRS11");
        dispensePivot = hardwareMap.get(Servo.class, "DPS12");
        dispenseGate = hardwareMap.get(Servo.class, "DGS13");

        slideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideExtend.setTargetPosition(0);
        slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRotate.setTargetPosition(0);
        slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideExtend.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode to make this JSON
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new BNO055IMU.Parameters().accelerationIntegrationAlgorithm;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        //detect TSE
        tse curCase = tse.TOP;
        manualDrive(1.0, 1.0, 0.5);
        moveArm(curCase);
        turnToHeading(270.0, Direction.LEFT, 0.2, 0.5, 4);
        manualDrive(1.0, 1.0, 6);
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
    public double clamp(double value, double min, double max){
        if(value < min)
            return min;
        else if(value > max)
            return max;
        else
            return value;
    }
}
