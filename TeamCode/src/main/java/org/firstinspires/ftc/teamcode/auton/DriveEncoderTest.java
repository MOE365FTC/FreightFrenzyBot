package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous
public class DriveEncoderTest extends LinearOpMode {
    DcMotor frontLeft, backLeft, frontRight, backRight;

    ElapsedTime timer = new ElapsedTime();
    public void hold(double waitTime) {
        timer.reset();
        while(timer.time() < waitTime && opModeIsActive()) {

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "TLM10");
        backLeft = hardwareMap.get(DcMotor.class, "BLM11");
        frontRight = hardwareMap.get(DcMotor.class, "TRM12");
        backRight = hardwareMap.get(DcMotor.class, "BRM13");
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(0);
//        backLeft.setTargetPosition(0);
//        frontRight.setTargetPosition(0);
        backRight.setTargetPosition(0);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        telemetry.addData("Started", "Yes");
        telemetry.update();
        hold(0.5);

        drive(-24);
//        drive(5.0);
//        turn(90);
//        drive(4.0);
//        drive(-35.0);
//        turn(-10);
//        drive(12.0);
//        turn(135);
//        drive(42);
    }

    public void drive(double inches) {
        while(Math.abs(frontLeft.getCurrentPosition()) < Math.abs((inches*38.1)) && opModeIsActive()) {
            frontLeft.setPower((inches/Math.abs(inches)) * 0.5);
            backLeft.setPower((inches/Math.abs(inches)) * 0.5);
            frontRight.setPower((inches/Math.abs(inches)) * 0.5);
            backRight.setPower((inches/Math.abs(inches)) * 0.5);
            telemetry.addData("inch", inches*38.1);
            telemetry.addData("encoder", frontLeft.getCurrentPosition());
            telemetry.update();
        }
        telemetry.addData("Drove", "Yes");
        telemetry.update();
    }

    public void turn(double degrees) {
        while(Math.abs(frontLeft.getCurrentPosition()) < (degrees*5.29) && opModeIsActive()) {
            frontLeft.setPower(0.5*(degrees/degrees));
            backLeft.setPower(0.5*(degrees/degrees));

            frontRight.setPower(0.5*(-degrees/degrees));
            backRight.setPower(0.5*(-degrees/degrees));
        }
    }
}

