package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Carousel {
    Gamepad gamepad1;
    CRServo spinner1, spinner2;

    final double spinPower = 1.0;

    public Carousel(HardwareMap hardwareMap, Gamepad gpad1){
        this.gamepad1 = gpad1;
        spinner1 = hardwareMap.crservo.get("RSS15");
        spinner2 = hardwareMap.crservo.get("BSS14");
    }

    public void actuate() {
        if(gamepad1.dpad_left) {
            spinner1.setPower(spinPower);
            spinner2.setPower(spinPower);
        } else if(gamepad1.dpad_right){
            spinner1.setPower(-spinPower);
            spinner2.setPower(-spinPower);
        } else{
            spinner1.setPower(0.0);
            spinner2.setPower(0.0);
        }
    }
}
