package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.functions.enums.slideSetting;
import org.firstinspires.ftc.teamcode.functions.enums.slideState;
import org.firstinspires.ftc.teamcode.teleop.OutputtingTeamMarkerPos;
import org.firstinspires.ftc.teamcode.teleop.TeamMarkerTracker;
import org.firstinspires.ftc.teamcode.test.AutonomousTurnTest;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

enum Direction{
    LEFT,
    RIGHT
}

enum tse{
        TOP,
        MID,
        BOT
}
@Autonomous
public class BlueTopAuton extends LinearOpMode {
    BNO055IMU imu;
    double HEADING_OFFSET = 270; // starting heading of the robot, change this for every auton starting pose

    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    DcMotorEx slideExtend, slideRotate;
    Servo dispenseTilt, dispensePivot, dispenseGate;
    DcMotor intake;
    CRServo spinner;
    AnalogInput limitSwitch0, limitSwitch1;
    double spinPower = 1.0;
    final int SLIDE_MULTIPLIER = 1;
    slideState curSlideState = slideState.RETRACTED;
    slideSetting curSlideSetting = slideSetting.RETRACT;
    final double SLIDE_RETRACT_POWER = -0.3, SLIDE_EXTEND_POWER = 0.8;
    int slideTargetPos = 0;
    final int tiltTicsFor90degrees = 1453; //Number of tics for 90 degrees of slide rotation (influences dispenser tilt)
    final int extendMinimum = 100; //Min slide extension tics before tilting dispenser or opening gate (must be above motor to prevent damage) (used for tilt and gate)
    final double tiltMinimum = 0.35 / 2; //Min dispenser tilt servo position before pivoting servo (must not turn into the slides) (used for pivot)
    final int ticsPerInch = 38;
    final double rotateTicsDeltaToVertical = 1180.0;

    ElapsedTime timer = new ElapsedTime();

    public void runToLocation(DcMotorEx motor, double speed, double target, int tolerance){
        int dir = 1; // assume we go forwards
        if(motor.getCurrentPosition() > target){ // now we need to go backwards
            dir = -1;
        }
        int error = (int) Math.abs(motor.getCurrentPosition() - target);
        motor.setPower(dir * speed);
        while(error > tolerance && opModeIsActive()){
            error = (int) Math.abs(motor.getCurrentPosition() - target);
        }
        motor.setPower(0);
    }
    public void wait(double waitTime) {
        timer.reset();
        while (timer.time() < waitTime && opModeIsActive()) {
        }
    }
    public slideState getSlideCurrentState() {
        if (limitSwitch0.getVoltage() > 1 && limitSwitch1.getVoltage() < 1) {
            return slideState.EXTENDED;
        } else if (limitSwitch0.getVoltage() < 1 && limitSwitch1.getVoltage() > 1) {
            return slideState.RETRACTED;
        } else {
            telemetry.addLine("LIMIT SWITCH DISCONNECTED");
            return slideState.ERROR;
        }
    }
    public void dispenser() {
        //Tilt:
        if (slideExtend.getCurrentPosition() > extendMinimum && curSlideSetting == slideSetting.EXTEND && curSlideState == slideState.EXTENDED) {
            dispenseTilt.setPosition((slideRotate.getCurrentPosition() / tiltTicsFor90degrees) * .35);
        } else {
            dispenseTilt.setPosition(0.0);
        }
        //Gate:
        if (slideExtend.getCurrentPosition() > extendMinimum && curSlideSetting == slideSetting.EXTEND && curSlideState == slideState.EXTENDED) {
            if (gamepad2.left_bumper) {
                dispenseGate.setPosition(0.45);
            } else {
                dispenseGate.setPosition(1.0);
            }
        } else {
            dispenseGate.setPosition(1.0);
        }
        //Pivot:
        if (dispenseTilt.getPosition() > tiltMinimum && slideExtend.getCurrentPosition() > extendMinimum && curSlideSetting == slideSetting.EXTEND && curSlideState == slideState.EXTENDED) {
            if (dpadLeft2Toggled) {
                dispensePivot.setPosition(0.8);
                dpadRight2Toggled = false;
                dpadUp2Toggled = false;
            } else if (dpadUp2Toggled) {
                dispensePivot.setPosition(0.0);
                dpadRight2Toggled = false;
                dpadLeft2Toggled = false;
            } else if (dpadRight2Toggled) {
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
        if (gamepad2.right_trigger > 0 && curSlideState == slideState.RETRACTED) {
            intake.setPower(gamepad2.right_trigger);
        } else if (gamepad2.right_bumper) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }
    }
    public void spinCarousel() {
        if (gamepad1.dpad_left) {
            spinner.setPower(spinPower);
        } else if (gamepad1.dpad_right) {
            spinner.setPower(spinPower);
        } else {
            spinner.setPower(0.0);
        }
    }
    public void manualDrive(double leftPower, double rightPower, double time) {
        motorBackLeft.setPower(leftPower);
        motorFrontLeft.setPower(leftPower);
        motorBackRight.setPower(rightPower);
        motorFrontRight.setPower(rightPower);
        sleep((long) (time * 1000));
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
    }
    public void forward(double distance, double power) {
        int target = (int) (distance * ticsPerInch) + motorFrontLeft.getCurrentPosition();
        int tolerance = 5;
        motorBackLeft.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorFrontRight.setPower(power);
        while(motorFrontLeft.getCurrentPosition() <= target-tolerance && opModeIsActive()){
        }
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
    }
    public void back(double distance, double power) {
        int target = motorFrontLeft.getCurrentPosition()- (int) (distance * ticsPerInch);
        int tolerance = 5;
        motorBackLeft.setPower(-power);
        motorFrontLeft.setPower(-power);
        motorBackRight.setPower(-power);
        motorFrontRight.setPower(-power);
        while(motorFrontLeft.getCurrentPosition() >= target-tolerance && opModeIsActive()){
        }
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
    }

    public void turnToHeading(double targetHeading, Direction dir, double turnPower, double tolerance) {
        //targetHeading: final heading to face
        //dir: Direction to turn
        //minPower: minimum power to turn the robot at
        //maxPower: max power to turn the robot at (if min == max, constant turning speed)
        //tolerance: tolerance (in degrees) to stop the turn at
        double turnScaling = 0.1; //TODO: coefficient to modify the error by, trial and error here
        double imuHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
        imuHeading = (imuHeading + 360) % 360;
        double curHeading = (imuHeading + HEADING_OFFSET) % 360; // this is our 'adjusted' current heading
        int multiplier = dir == Direction.RIGHT ? 1 : -1; // multiply powers by 1 if RIGHT, -1 if LEFT to get the turn direction
        double error = Math.abs(curHeading - targetHeading);
        motorFrontRight.setPower(-multiplier * turnPower);
        motorBackRight.setPower(-multiplier * turnPower);
        motorFrontLeft.setPower(multiplier * turnPower);
        motorBackLeft.setPower(multiplier * turnPower);
        while (error > tolerance && opModeIsActive()) { // ALWAYS put an opModeIsActive() check in all FTC while loops!!!
            imuHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            imuHeading = (imuHeading + 360) % 360;
            curHeading = (imuHeading + HEADING_OFFSET) % 360;
            error = Math.abs(curHeading - targetHeading);
//            turnPower = clamp(error * turnScaling, minPower, maxPower);

//            sleep(10); // sleep for 10 milliseconds, we can tune this number too.
            telemetry.addData("error", error);
            telemetry.update();
        }
        stopDrive();


    }
    public void stopDrive() {
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
    }

    public void moveArm(int rotPos, int extPos){
        runToLocation(slideRotate, 0.5, rotPos, 10);
        slideExtend.setTargetPosition(extPos);
        slideExtend.setPower(0.8);
    }
    WebcamName webcamName;
    OpenCvCamera camera;
    TeamMarkerTracker tracker;
    public void runOpMode() {
        tracker = new TeamMarkerTracker();
        webcamName = hardwareMap.get(WebcamName.class, "TSECam");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName);
        tracker = new TeamMarkerTracker();
        camera.openCameraDevice();
        camera.startStreaming(160, 90
                , OpenCvCameraRotation.UPRIGHT);
        camera.setPipeline(tracker);
        motorFrontLeft = hardwareMap.dcMotor.get("TLM10");
        motorBackLeft = hardwareMap.dcMotor.get("BLM11");
        motorFrontRight = hardwareMap.dcMotor.get("TRM12");
        motorBackRight = hardwareMap.dcMotor.get("BRM13");

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

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
        slideRotate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        slideRotate.setTargetPosition(0);
//        slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideExtend.setDirection(DcMotor.Direction.REVERSE);
        slideRotate.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideRotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideRotate.setPositionPIDFCoefficients(-0.1);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode to make this JSON
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new BNO055IMU.Parameters().accelerationIntegrationAlgorithm;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("STATUS", "IMU Initializing...");
        telemetry.update();
        imu.initialize(parameters);
        telemetry.addData("STATUS", "Ready!");
        telemetry.update();

        waitForStart();
        tse curCase = tse.TOP;
        telemetry.addData("xpos", tracker.xpos);
        switch(tracker.position){
            case 1:
                curCase = tse.BOT;
                break;
            case 2:
                curCase = tse.MID;
                break;
            case 3:
                curCase = tse.TOP;
                break;
            default:
                curCase = tse.TOP;
                break;
        }
        telemetry.addData("case", curCase);
        telemetry.update();
        //top: rot=857 ext=1580
        //mid: rot=857 ext=780 **BUT MOVE FORWARD MORE**
        //low: rot=857 ext=780 **BUT DONT MOVE FORWARD**
        //detect TSE
        forward(24, 0.5);
        turnToHeading(210.0, Direction.RIGHT, 0.3, 2);
        switch(curCase) {
            case TOP:
                forward(1, 0.3);
                moveArm(2650, 1580);
                while((slideExtend.isBusy() || slideRotate.isBusy()) && opModeIsActive()){
                }
                dispenseTilt.setPosition(0.144);
                dispenseGate.setPosition(0.45);
                sleep(1000);
                dispenseGate.setPosition(1.0);
                back(3, 0.5);
                break;
            case MID:
                forward(2.5, 0.3);
                moveArm(2800,780);
                while(slideExtend.isBusy() || slideRotate.isBusy() && opModeIsActive()){
                }
                dispenseTilt.setPosition(0.22);
                dispenseGate.setPosition(0.45);
                sleep(1000);
                dispenseGate.setPosition(1.0);
                back(2.5, 0.5);
                break;
            case BOT:
                back(1.0, 0.5);
                moveArm(2950,780);
                while(slideExtend.isBusy() || slideRotate.isBusy() && opModeIsActive()){
                }
                dispenseTilt.setPosition(0.24);
                dispenseGate.setPosition(0.45);
                sleep(1000);
                dispenseGate.setPosition(1.0);
                back(1.5, 0.5);
                break;
            default:
                break;
        }
        slideState curSlideState = getSlideCurrentState();
        dispenseTilt.setPosition(0.0);
        runToLocation(slideRotate, 0.5, rotateTicsDeltaToVertical, 10);
        slideExtend.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(curSlideState == slideState.EXTENDED && opModeIsActive()){
            curSlideState = getSlideCurrentState();
            slideExtend.setPower(SLIDE_RETRACT_POWER);
        }
        slideExtend.setPower(0.0);
        while(slideRotate.isBusy() && opModeIsActive()){

        }
        turnToHeading(190.0, Direction.RIGHT, 0.3, 2);
        manualDrive(-0.8, -0.8, 1.1);
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
