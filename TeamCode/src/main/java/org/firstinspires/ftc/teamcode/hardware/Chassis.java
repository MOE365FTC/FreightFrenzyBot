package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.TurnDirection;

public class Chassis {
    DcMotor frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor;
    Servo odoLeftServo, odoRightServo;
    Gamepad gamepad1;
    LinearOpMode opMode;
    double scaleFactor, leftPower, rightPower;

    // Set heading offset in autonomous
    int headingOffset;

    // drive variables
    final double driveSpeed = 0.6;
    double heading = headingOffset;
    final int ticsPerInch = 1450;
    final int turnTicsPerDegree = 310;
    final double turnSlowPower = 0.2;

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

    public void forward_inches(double inches, double power){
        int target = (int) (inches * ticsPerInch) + frontRightMotor.getCurrentPosition();
        int tolerance = 5;
        backLeftMotor.setPower(power);
        frontLeftMotor.setPower(power);
        backRightMotor.setPower(power);
        frontRightMotor.setPower(power);
        while(frontRightMotor.getCurrentPosition() <= target-tolerance && this.opMode.opModeIsActive()){

        }
        this.stopDrive();
    }

    public void backward_inches(double inches, double power){
        int target = frontRightMotor.getCurrentPosition() - (int)(inches * ticsPerInch);
        int tolerance = 5;
        backLeftMotor.setPower(-power);
        frontLeftMotor.setPower(-power);
        backRightMotor.setPower(-power);
        frontRightMotor.setPower(-power);
        while(frontRightMotor.getCurrentPosition() >= target-tolerance && this.opMode.opModeIsActive()){
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
            if (error < 30)
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
        telemetry.addData("left", frontLeftMotor.getCurrentPosition());
        telemetry.addData("right", frontRightMotor.getCurrentPosition());
        telemetry.addData("sum", sum);
        telemetry.addData("scaled", sum / this.turnTicsPerDegree);
        telemetry.addData("heading", this.getHeading());
    }

}
