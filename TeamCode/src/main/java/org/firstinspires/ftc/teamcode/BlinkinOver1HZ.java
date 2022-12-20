package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="BlinkinOver1HZ", group="A")
public class BlinkinOver1HZ extends DriveMethods {
    @Override
    public void runOpMode() {
        initBlinkinOnly();
        waitForStart();
        while(opModeIsActive()) {
            if(Variables.pattern == RevBlinkinLedDriver.BlinkinPattern.GREEN) {
                setBlinkinColor(Variables.BlinkinColor.YELLOW);
            } else {
                setBlinkinColor(Variables.BlinkinColor.GREEN);
            }
        }
    }
}
