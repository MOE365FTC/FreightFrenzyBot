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
    final double ticsPerInch = 1650;
    public static double backwardTicsPerInch = 1720;
    public static double kp = 0.000013;
    public static double ki = 2e-5;
    public static double integralCap = 200;

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

    //Autonomous constructor (saves LinearOpMode then calls first constructor)
    public Chassis(HardwareMap hardwareMap, Gamepad gpad1, LinearOpMode opMode, int headingOffset){
        this(hardwareMap, gpad1);
        this.opMode = opMode;
        this.headingOffset = headingOffset;
        this.setOdometryDown(true); // lower odometry
        this.zeroEncoders();
    }

    public void actuate(){
        scaleFactor = driveSpeed + gamepad1.left_trigger * (1-driveSpeed);
        leftPower = -gamepad1.left_stick_y * scaleFactor;
        rightPower = -gamepad1.right_stick_y * scaleFactor;
        frontLeftMotor.setPower(leftPower);
        backLeftMotor.setPower(leftPower);
        frontRightMotor.setPower(rightPower);
        backRightMotor.setPower(rightPower);
    }

    public void stopDrive() {
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
    }

    public void setOdometryDown(boolean down){
        if (down){
            odoRightServo.setPosition(odometryRightDownPos);
            odoLeftServo.setPosition(odometryLeftDownPos);
        } else {
            odoRightServo.setPosition(odometryRightUpPos);
            odoLeftServo.setPosition(odometryLeftUpPos);
        }
    }

    public void zeroEncoders(){
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public double getHeading(){
        // get our heading from the axial encoders
        // return in degrees
        double raw = (frontRightMotor.getCurrentPosition() - frontLeftMotor.getCurrentPosition()) / turnTicsPerDegree;
        return clampDegrees(raw + headingOffset);
    }

    public void forward_inches(double inches, double tolerance){
        double target = (inches * ticsPerInch) + frontRightMotor.getCurrentPosition();
        double error = target - frontRightMotor.getCurrentPosition();
        double power = 0;
        double errorSum = 0;
        while(this.opMode.opModeIsActive() && (Math.abs(error) > tolerance * ticsPerInch)){
            error = target - frontRightMotor.getCurrentPosition();
            power = error * this.kp + errorSum * this.ki;;
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("currentPos", frontRightMotor.getCurrentPosition() / this.ticsPerInch);
            packet.put("target", inches);
            packet.put("power", power);
            packet.put("error", error);
            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(packet);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(power);
            errorSum += error;
            if (this.integralCap > 0 && Math.abs(errorSum) > this.integralCap){
                errorSum = Math.signum(errorSum) * this.integralCap;
            }
        }
        this.stopDrive();
    }

    public void backward_inches(double inches, double tolerance){
        double target = frontRightMotor.getCurrentPosition() - (inches * backwardTicsPerInch);
        double error = target - frontRightMotor.getCurrentPosition();
        double power = 0;
        double errorSum = 0;
        while(this.opMode.opModeIsActive() && (Math.abs(error) > tolerance * backwardTicsPerInch)){
            error = target - frontRightMotor.getCurrentPosition();
            power = error * this.kp + errorSum * this.ki;
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("currentPos", frontRightMotor.getCurrentPosition() / this.backwardTicsPerInch);
            packet.put("target", -inches);
            packet.put("power", power);
            packet.put("error", error);
            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(packet);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
            frontLeftMotor.setPower(power);
            backLeftMotor.setPower(power);
            errorSum += error;
            if (this.integralCap > 0 && Math.abs(errorSum) > this.integralCap){
                errorSum = Math.signum(errorSum) * this.integralCap;
            }
        }
        this.stopDrive();
    }

    public void driveSeconds(double power, double time) {
        backLeftMotor.setPower(power);
        frontLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontRightMotor.setPower(power);
        this.opMode.sleep((long) (time * 1000));
        stopDrive();
    }

    public void setLeftPower(double power, double time){
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        this.opMode.sleep((long) time * 1000);
        stopDrive();
    }

    public void setRightPower(double power, double time){
        backLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        this.opMode.sleep((long) time * 1000);
        stopDrive();
    }

    public void turnToHeading(double targetHeading, TurnDirection dir, double turnPower, double tolerance){
        double heading = this.getHeading();
        int multiplier = dir == TurnDirection.RIGHT ? 1 : -1;
        double error = Math.abs(heading - targetHeading);

        while (error > tolerance && this.opMode.opModeIsActive()){
            if (error < 10)
                turnPower = turnSlowPower;
            frontRightMotor.setPower(-multiplier * turnPower);
            backRightMotor.setPower(-multiplier * turnPower);
            frontLeftMotor.setPower(multiplier * turnPower);
            backLeftMotor.setPower(multiplier * turnPower);
            heading = this.getHeading();
            error = Math.abs(heading - targetHeading);
        }

        stopDrive();
    }

    public void turn(float targetHeading, float tolerance){
        long startTime = System.nanoTime();
        long curTime = System.nanoTime();
        long lastTime;
        double lastError = 0;
        double errorSum = 0;
        double error = targetHeading - getHeading();
        error = (int)(error/180) * -360 + error;
        double errorSlope = 0;
        while(this.opMode.opModeIsActive() && (Math.abs(error) > tolerance || Math.abs(errorSlope) > slopeTolerance)){
            lastTime = curTime;
            curTime = System.nanoTime();

            error = targetHeading - getHeading();
            error = (int)(error/180) * -360 + error;
            errorSlope = (error - lastError) / (curTime - lastTime);
            double power = error * this.turnKp + errorSum * this.turnKi + errorSlope * this.turnKd;
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("currentHeading", getHeading());
            packet.put("targetHeading", targetHeading);
            packet.put("power", power);
            packet.put("curTime", (curTime-startTime)/1000000000f);
            packet.put("errorSum", errorSum);
            FtcDashboard dashboard = FtcDashboard.getInstance();
            dashboard.sendTelemetryPacket(packet);
            frontRightMotor.setPower(power);
            backRightMotor.setPower(power);
            frontLeftMotor.setPower(-power);
            backLeftMotor.setPower(-power);
            errorSum += error;
            if (this.turnIntegralCap > 0 && Math.abs(errorSum) > this.turnIntegralCap){
                errorSum = Math.signum(errorSum) * this.turnIntegralCap;

            }
            lastError = error;
        }
        this.stopDrive();
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
        frontRightMotor.setPower(-multiplier * turnPower);
        backRightMotor.setPower(-multiplier * turnPower);
        frontLeftMotor.setPower(multiplier * turnPower);
        backLeftMotor.setPower(multiplier * turnPower);
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
        telemetry.addData("left", frontLeftMotor.getCurrentPosition());
        telemetry.addData("right", frontRightMotor.getCurrentPosition() / backwardTicsPerInch);
        telemetry.addData("sum", sum);
        telemetry.addData("scaled", sum / this.turnTicsPerDegree);
        telemetry.addData("heading", this.getHeading());
    }

}
