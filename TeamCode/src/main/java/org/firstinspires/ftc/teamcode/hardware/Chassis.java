package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.TurnDirection;

@Config
public class Chassis {
    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    Servo odoLeftServo, odoRightServo;
//    Rev2mDistanceSensor distanceSensor;
    Gamepad gamepad1;
    LinearOpMode opMode;
    double scaleFactor, leftPower, rightPower;

    // Set heading offset in autonomous
    int headingOffset;

    // drive variables
    final double driveSpeed = 0.6;
    public static double ticsPerInch_r = 1730;
    public static double backwardTicsPerInch_r = 1720;
    public static double ticsPerInch_l = 1670;
    public static double backwardTicsPerInch_l = 1700;
    public static double kp = 12e-6;
    public static double ki = 9e-6;
    public static double kd = 0;
    public static double integralCap = 1e4;

    // turning variables
    double heading = headingOffset;
    final double turnTicsPerDegree = 310;
    final double turnSlowPower = 0.2;
    public static double turnKp = 0.015; //0.012
    public static  double turnKi = 0.0011; //0.001
    public static  double turnKd = 7000000; //2000000
    public static  double turnIntegralCap = 200;
    public static double slopeTolerance = 0.000000028;

    final double odometryLeftUpPos = 1.0;
    final double odometryLeftDownPos = 0.0;
    final double odometryRightUpPos = 0.0;
    final double odometryRightDownPos = 1.0;

    //Teleop constructor (no LinearOpMode reference)
    public Chassis(HardwareMap hardwareMap, Gamepad gpad1){
        this.gamepad1 = gpad1;
        frontLeftMotor =  hardwareMap.get(DcMotor.class, "FLM10");
        backLeftMotor =   hardwareMap.get(DcMotor.class, "BLM11");
        frontRightMotor = hardwareMap.get(DcMotor.class, "FRM00");
        backRightMotor =  hardwareMap.get(DcMotor.class, "BRM01");

        odoRightServo = hardwareMap.get(Servo.class, "ORS05");
        odoLeftServo = hardwareMap.get(Servo.class, "OLS15");

//        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "RDS02");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.setOdometryDown(false); // raise odometry
        this.zeroEncoders();
    }

//    public Chassis(HardwareMap hardwareMap, Gamepad gpad1) {
//        frontLeft = hardwareMap.get(DcMotor.class, "FLM10");
//        backLeft = hardwareMap.get(DcMotor.class, "BLM11");
//        odoServoLeft = hardwareMap.get(Servo.class, "OLS15");
//
//        frontRight = hardwareMap.get(DcMotor.class, "FRM00");
//        backRight = hardwareMap.get(DcMotor.class, "BRM01");
//        odoServoRight = hardwareMap.get(Servo.class, "ORS05");
//
//        leftDrive = new DriveSideLeft(frontLeft, backLeft, odoServoLeft, true,  false, 1.0, 0.0, ticsPerInch_l, backwardTicsPerInch_l);
//        rightDrive = new DriveSideRight(frontRight, backRight, odoServoRight, false, true,  0.0, 1.0, ticsPerInch_r, backwardTicsPerInch_r);
//
//
//        this.gamepad1 = gpad1;
//    }

    //Autonomous constructor (saves LinearOpMode then calls first constructor)
    public Chassis(HardwareMap hardwareMap, Gamepad gpad1, LinearOpMode opMode, int headingOffset){
        this(hardwareMap, gpad1);
        this.opMode = opMode;
        this.headingOffset = headingOffset;
        this.setOdometryDown(true); // lower odometry
        this.zeroEncoders();
    }
    public void setRightPower(double power){
        frontRightMotor.setPower(power);
        backRightMotor.setPower(power);
    }

    public void setLeftPower(double power){
        frontLeftMotor.setPower(power);
        backLeftMotor.setPower(power);
    }

    public void actuate() {
        scaleFactor = driveSpeed + gamepad1.left_trigger * (1 - driveSpeed);
        leftPower = -gamepad1.left_stick_y * scaleFactor;
        rightPower = -gamepad1.right_stick_y * scaleFactor;
        setRightPower(rightPower);
        setLeftPower(leftPower);
    }

    public void stopDrive() {
        setLeftPower(0.0);
        setRightPower(0.0);
    }

    public void setOdometryDown(boolean isDown) {
        if(isDown) {
            odoLeftServo.setPosition(odometryLeftDownPos);
            odoRightServo.setPosition(odometryRightDownPos);
        } else {
            odoLeftServo.setPosition(odometryLeftUpPos);
            odoRightServo.setPosition(odometryRightUpPos);
        }
    }

    public void zeroEncoders(){
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double clampDegrees(double raw){
        // take a raw degree input and wrap it to be between 0 and 360
        while(raw > 360){
            raw -= 360;
        }
        while(raw < 0){
            raw += 360;
        }
        return raw;
    }

    public double clampErrorSum(double errorSum) {
        if (this.integralCap > 0 && Math.abs(errorSum) > this.integralCap)
            return errorSum = Math.signum(errorSum) * this.integralCap;
        else
            return errorSum;
    }

    public double getHeading() {
        // get our heading from the axial encoders
        // return in degrees
        double raw = (frontRightMotor.getCurrentPosition() - frontLeftMotor.getCurrentPosition()) / turnTicsPerDegree;
        return clampDegrees(raw + headingOffset);
    }

    public void forward_inches(double inches, double tolerance) {
        double tolerance_r = tolerance * ticsPerInch_r; // update tolerance to be in ticsPerInch instead of inches
        double tolerance_l = tolerance * ticsPerInch_l;

        // rightDrive PID init
        double target_r = (inches * ticsPerInch_r) + frontRightMotor.getCurrentPosition();
        double error_r = target_r - frontRightMotor.getCurrentPosition();
        double power_r = 0;
        double errorSum_r = 0;
        double lastError_r = 0;
        double errorSlope_r = 0;

        // leftDrive PID init
        double target_l = (inches * ticsPerInch_l) + frontLeftMotor.getCurrentPosition();
        double error_l = target_l - frontLeftMotor.getCurrentPosition();
        double power_l = 0;
        double errorSum_l = 0;
        double lastError_l = 0;
        double errorSlope_l = 0;

        long curTime = System.nanoTime();
        long lastTime;

        while (this.opMode.opModeIsActive() && (Math.abs(error_r) > tolerance_r || Math.abs(error_l) > tolerance_l)) {
            lastTime = curTime;
            curTime = System.nanoTime();

            error_r = target_r - frontRightMotor.getCurrentPosition();
            error_l = target_l - frontLeftMotor.getCurrentPosition();

            errorSlope_r = (error_r - lastError_r) / (curTime - lastTime);
            errorSlope_l = (error_l - lastError_l) / (curTime - lastTime);
            power_r = error_r * this.kp + errorSum_r * this.ki  + errorSlope_r * this.kd;
            power_l = error_l * this.kp + errorSum_l * this.ki  + errorSlope_l * this.kd;

            // FTC Dashboard Telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("currentPos_r", frontRightMotor.getCurrentPosition() / ticsPerInch_r);
            packet.put("currentPos_l", frontLeftMotor.getCurrentPosition() / ticsPerInch_l);
            packet.put("target", inches);
            packet.put("power_r", power_r);
            packet.put("error_r", error_r);
            packet.put("error_sum_r", errorSum_r);
            packet.put("error_slope_r", errorSlope_r);
            packet.put("power_l", power_l);
            packet.put("error_l", error_l);
            packet.put("error_sum_l", errorSum_l);
            packet.put("error_slope_l", errorSlope_l);
            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(packet);
            setRightPower(power_r);
            setLeftPower(power_l);
            errorSum_r += error_r;
            errorSum_l += error_l;
            errorSum_r = clampErrorSum(errorSum_r);
            errorSum_l = clampErrorSum(errorSum_l);

            lastError_r = error_r;
            lastError_l = error_l;
        }
        this.stopDrive();
    }

    public void backward_inches(double inches, double tolerance) {
        double tolerance_r = tolerance * backwardTicsPerInch_r; // update tolerance to be in tics not inches
        double tolerance_l = tolerance * backwardTicsPerInch_l;

        // rightDrive PID init
        double target_r = frontRightMotor.getCurrentPosition() - (inches * backwardTicsPerInch_r);
        double error_r = target_r - frontRightMotor.getCurrentPosition();
        double power_r = 0;
        double errorSum_r = 0;
        double lastError_r = 0;
        double errorSlope_r = 0;

        // leftDrive PID init
        double target_l = frontLeftMotor.getCurrentPosition() - (inches * backwardTicsPerInch_l);
        double error_l = target_l - frontLeftMotor.getCurrentPosition();
        double power_l = 0;
        double errorSum_l = 0;
        double lastError_l = 0;
        double errorSlope_l = 0;

        long curTime = System.nanoTime();
        long lastTime;

        while (this.opMode.opModeIsActive() && (Math.abs(error_r) > tolerance_r || Math.abs(error_l) > tolerance_l)) {
            lastTime = curTime;
            curTime = System.nanoTime();

            error_r = target_r - frontRightMotor.getCurrentPosition();
            error_l = target_l - frontLeftMotor.getCurrentPosition();

            errorSlope_r = (error_r - lastError_r) / (curTime - lastTime);
            errorSlope_l = (error_l - lastError_l) / (curTime - lastTime);

            power_r = error_r * this.kp + errorSum_r * this.ki + errorSlope_r * this.kd;;
            power_l = error_l * this.kp + errorSum_l * this.ki + errorSlope_l * this.kd;;

            // FTC Dashboard Telemetry
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("currentPos_r", frontRightMotor.getCurrentPosition() / backwardTicsPerInch_r);
            packet.put("currentPos_l", frontLeftMotor.getCurrentPosition() / backwardTicsPerInch_l);
            packet.put("target", inches);
            packet.put("power_r", power_r);
            packet.put("error_r", error_r);
            packet.put("power_l", power_l);
            packet.put("error_l", error_l);
            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(packet);

            setLeftPower(power_l);
            setRightPower(power_r);
            errorSum_r += error_r;
            errorSum_l += error_l;
            errorSum_r = clampErrorSum(errorSum_r);
            errorSum_l = clampErrorSum(errorSum_l);

            lastError_r = error_r;
            lastError_l = error_l;
        }
        this.stopDrive();
    }

    public void driveSeconds(double power, double time) {
        setLeftPower(power);
        setRightPower(power);
        this.opMode.sleep((long) (time * 1000));
        stopDrive();
    }

    public void turn(float targetHeading, float tolerance) {
        long startTime = System.nanoTime();
        long curTime = System.nanoTime();
        long lastTime;
        double lastError = 0;
        double errorSum = 0;
        double error = targetHeading - getHeading();
        error = (int) (error / 180) * -360 + error;
        double errorSlope = 0;
        while (this.opMode.opModeIsActive() && (Math.abs(error) > tolerance || Math.abs(errorSlope) > slopeTolerance)) {
            lastTime = curTime;
            curTime = System.nanoTime();

            error = targetHeading - getHeading();
            error = (int) (error / 180) * -360 + error;
            errorSlope = (error - lastError) / (curTime - lastTime);
            double power = error * this.turnKp + errorSum * this.turnKi + errorSlope * this.turnKd;
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("currentHeading", getHeading());
            packet.put("targetHeading", targetHeading);
            packet.put("power", power);
            packet.put("curTime", (curTime - startTime) / 1000000000f);
            packet.put("errorSum", errorSum);
            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(packet);
            setLeftPower(-power);
            setRightPower(power);
            errorSum += error;
            if (this.turnIntegralCap > 0 && Math.abs(errorSum) > this.turnIntegralCap) {
                errorSum = Math.signum(errorSum) * this.turnIntegralCap;

            }
            lastError = error;
        }
        this.stopDrive();
    }

    @Deprecated
    public void turnToHeading(double targetHeading, TurnDirection dir, double turnPower, double tolerance){
        double heading = this.getHeading();
        int multiplier = dir == TurnDirection.RIGHT ? 1 : -1;
        double error = Math.abs(heading - targetHeading);

        while (error > tolerance && this.opMode.opModeIsActive()){
            if (error < 10)
                turnPower = turnSlowPower;
            setRightPower(-multiplier * turnPower);
            setLeftPower(multiplier * turnPower);
            heading = this.getHeading();
            error = Math.abs(heading - targetHeading);
        }

        stopDrive();
    }

    @Deprecated
    public void turnToHeadingIMU(double targetHeading, TurnDirection dir, double turnPower, double tolerance) {
        double turnScaling = 0.1; //TODO: coefficient to modify the error by, trial and error here
//        double imuHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
        double imuHeading = 0.0; //TODO: Get from encoders
        imuHeading = (imuHeading + 360) % 360;
        double curHeading = (imuHeading + headingOffset) % 360; // this is our 'adjusted' current heading
        int multiplier = dir == TurnDirection.RIGHT ? 1 : -1; // multiply powers by 1 if RIGHT, -1 if LEFT to get the turn direction
        double error = Math.abs(curHeading - targetHeading);
        setRightPower(-multiplier * turnPower);
        setLeftPower(multiplier * turnPower);
        while (error > tolerance && this.opMode.opModeIsActive()) {
//            imuHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; //TODO: Get from encoders
            imuHeading = (imuHeading + 360) % 360;
            curHeading = (imuHeading + headingOffset) % 360;
            error = Math.abs(curHeading - targetHeading);
        }
        stopDrive();
    }

    public void composeTelemetry(Telemetry telemetry){
        double sum = frontRightMotor.getCurrentPosition() + frontLeftMotor.getCurrentPosition();
//        telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("left", frontLeftMotor.getCurrentPosition() / backwardTicsPerInch_l);
        telemetry.addData("right", frontRightMotor.getCurrentPosition() / backwardTicsPerInch_r);
        telemetry.addData("sum", sum);
        telemetry.addData("scaled", sum / this.turnTicsPerDegree);
        telemetry.addData("heading", this.getHeading());
    }

}
