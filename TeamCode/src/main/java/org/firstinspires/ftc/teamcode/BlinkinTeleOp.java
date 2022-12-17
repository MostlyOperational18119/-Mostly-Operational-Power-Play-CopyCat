package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="BlinkinTest")
public class BlinkinTeleOp extends LinearOpMode {
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;
    @Override
    public void runOpMode() {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        telemetry.addLine("Initialized LED driver.");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            if(gamepad1.a) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_GREEN);
            } if(gamepad1.b) {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
            }
            telemetry.addLine(pattern.name());
            telemetry.update();
        }
    }
}
