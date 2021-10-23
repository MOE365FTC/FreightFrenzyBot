package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp
public class IntakeRPM extends OpMode {

    DcMotorEx intake;

    @Override
    public void init() {
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            intake.setPower(0.8);
        }
        if (gamepad1.b){
            intake.setVelocity(TestConstants.RPM);
        }
        telemetry.addData( "velocity", intake.getVelocity());
        telemetry.addData("target velo", TestConstants.RPM);
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("velo", intake.getVelocity());

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);

    }
}
