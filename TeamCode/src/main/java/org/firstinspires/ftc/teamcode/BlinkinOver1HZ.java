package org.firstinspires.ftc.teamcode;

import static java.lang.Math.random;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="BlinkinOver1HZ", group="A")
public class BlinkinOver1HZ extends DriveMethods {
    int currentRandNum = 0;

    public int intrng(int minNum, int maxNum) {
        return (int)((random() * maxNum) + minNum);
    }

    @Override
    public void runOpMode() {
        initBlinkinOnly();
        waitForStart();

        telemetry.addLine("\"Oops\", accidentally created this.");
        telemetry.update();
        resetRuntime();


        while(opModeIsActive()) {
            currentRandNum = intrng(0,6);
            switch(currentRandNum) {
                case 0:
                    setBlinkinColor(Variables.BlinkinColor.GREEN);
                    break;
                case 1:
                    setBlinkinColor(Variables.BlinkinColor.YELLOW);
                    break;
                case 2:
                    setBlinkinColor(Variables.BlinkinColor.BLUE);
                    break;
                case 3:
                    setBlinkinColor(Variables.BlinkinColor.RED);
                    break;
                case 4:
                    setBlinkinColor(Variables.BlinkinColor.ORANGE);
                    break;
                case 5:
                    setBlinkinColor(Variables.BlinkinColor.PINK);
                    break;
                case 6:
                    setBlinkinColor(Variables.BlinkinColor.RAINBOW);
                    break;
            }
            if(getRuntime() > 15.0) {
                telemetry.addLine("\"Oops\", accidentally created this.");
                telemetry.addLine("Have your eyes melted yet?");
                telemetry.update();
            }

        }

    }
}
