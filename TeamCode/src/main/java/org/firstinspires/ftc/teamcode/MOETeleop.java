package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

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

    public void intake(){
        if(Toggle.aToggled){
            intake.setPower(INTAKE_POWER);
        } else if (intake.getCurrent(CurrentUnit.MILLIAMPS) > INTAKE_CURRENT) {
            intake.setPower(0.0);
            Toggle.aToggled = !Toggle.aToggled;
            holding = true;
        }
        else {
            intake.setPower(0.0);
        }
    }

    public void outtake(){
        holding = false;
    }

}
