package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class MOETeleop extends MOEOpMode {

    @Override
    public void init() {

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        chassis();

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
    }
}
