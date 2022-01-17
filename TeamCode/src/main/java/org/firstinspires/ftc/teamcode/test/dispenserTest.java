package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class dispenserTest extends OpMode {
    Servo dispenser;
    double pos = 0;
    boolean rbPress = false;
    boolean lbPress = false;
    @Override
    public void init() {
        dispenser = hardwareMap.get(Servo.class, "DGS13");
    }

    //pitch 0.00 (vertical) -> 0.35 (90 degrees)
    //yaw 0.15 (90 degrees CW)
    //yaw 0.48 (middle)
    //yaw 0.80 (90 degrees CCW)
    //dispense 0.45 (dispensing position like 30 degrees)
    //dispense 1.00 (intake position)

    @Override
    public void loop() {
        if(gamepad1.right_bumper && !rbPress){
            pos+= 0.05;
            rbPress = true;
        } else if(gamepad1.left_bumper && !lbPress){
            pos-= 0.05;
            lbPress = true;
        }

        if(rbPress && !gamepad1.right_bumper){
            rbPress = false;
        }
        if(lbPress && !gamepad1.left_bumper){
            lbPress = false;
        }
        dispenser.setPosition(pos);
        telemetry.addData("Position", pos);
    }
}
