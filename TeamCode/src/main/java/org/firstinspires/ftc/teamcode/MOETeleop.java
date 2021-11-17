package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.tuning.Carousel;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp
public class MOETeleop extends MOEOpMode {


    MOEOpMode robot = new MOEOpMode();
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        chassis();
        Toggle.handleToggles();
        intake();
        outtake();
        manualSlides();
        carousel();
        presetSlides();
    }

    public void chassis() {
        float drivePower = -gamepad1.left_stick_y;
        float rotatePower = gamepad1.right_stick_x;

        frontLeft.setPower(drivePower + rotatePower);
        frontRight.setPower(drivePower - rotatePower);
        backLeft.setPower(drivePower + rotatePower);
        backRight.setPower(drivePower - rotatePower);
    }

    public void manualSlides() {
        if(gamepad2.a){
            slideRotate.setTargetPosition((int)gamepad2.right_stick_y * SLIDE_MULTIPLIER);
            slideRotate.setPower(0.8);
            wait(0.5);

            slideExtend.setTargetPosition((int)gamepad2.left_stick_y * SLIDE_MULTIPLIER);
            slideExtend.setPower(0.8);
            wait(0.5);
        }
    }

    public void presetSlides() {
        int stage = 3;
        if(Toggle.dpadDownToggled && !Toggle.dpadLeftToggled && !Toggle.dpadUpToggled){
            stage = 1;
        } else if(Toggle.dpadLeftToggled && !Toggle.dpadDownToggled && !Toggle.dpadUpToggled){
            stage = 2;
        } else if(Toggle.dpadUpToggled && !Toggle.dpadDownToggled && !Toggle.dpadLeftToggled){
            stage = 3;
        } else {
            stage = 3;
        }
        if (stage == 1 && Toggle.bToggled){
            slideExtend.setTargetPosition(LOW_EXTEND);
            slideRotate.setTargetPosition(LOW_ROTATE);
            slideRotate.setPower(1.0);
            slideExtend.setPower(1.0);
            wait(2.0);
            slideRotate.setPower(0.0);
            slideExtend.setPower(0.0);
        } else if (stage == 2 && Toggle.bToggled){
            slideExtend.setTargetPosition(MID_EXTEND);
            slideRotate.setTargetPosition(MID_ROTATE);
            slideRotate.setPower(1.0);
            slideExtend.setPower(1.0);
            wait(2.0);
            slideRotate.setPower(0.0);
            slideExtend.setPower(0.0);
        } else if (stage == 3 && Toggle.bToggled) {
            slideExtend.setTargetPosition(HIGH_EXTEND);
            slideRotate.setTargetPosition(HIGH_ROTATE);
            slideRotate.setPower(1.0);
            slideExtend.setPower(1.0);
            wait(2.0);
            slideRotate.setPower(0.0);
            slideExtend.setPower(0.0);
        } else{
            slideExtend.setTargetPosition(STORED_EXTEND);
            slideRotate.setTargetPosition(STORED_ROTATE);
            slideRotate.setPower(1.0);
            slideExtend.setPower(1.0);
            wait(2.0);
            slideRotate.setPower(0.0);
            slideExtend.setPower(0.0);
        }

    }

    public void intake(){
        if(Toggle.yToggled){
            intake.setPower(INTAKE_POWER);
        } else if (intake.getCurrent(CurrentUnit.MILLIAMPS) > INTAKE_CURRENT) {
            intake.setPower(0.0);
            Toggle.yToggled = !Toggle.yToggled;
            holding = true;
        }
        else {
            intake.setPower(0.0);
        }
    }

    public void outtake(){
        if (Toggle.xToggled){
            holding = false;
            outtake.setPosition(OUTTAKE_OPEN);
        } else {
            outtake.setPosition(OUTTAKE_CLOSED);
        }
    }

    public void carousel(){
        if (Toggle.y2Toggled){
            carousel.setPower(CAROUSEL_POWER);
        } else {
            carousel.setPower(0.0);
        }
    }
}
