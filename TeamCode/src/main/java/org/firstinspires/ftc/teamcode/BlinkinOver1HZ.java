package org.firstinspires.ftc.teamcode;

import static java.lang.Math.random;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous(name="BlinkinOver1HZ", group="A")
@Disabled
public class BlinkinOver1HZ extends DriveMethods {
    int currentRandNum = 0;
    int rngdegnum = 69;
    public int intrng(int minNum, int maxNum) {
        return (int)((random() * maxNum) + minNum);
    }
    public double doublerng(int minNum, int maxNum) {
        return (random() * maxNum) + minNum;
    }


    @Override
    public void runOpMode() {
        initMotorsBlue();
//        while(opModeInInit()) {
//            currentRandNum = intrng(0,6);
//            switch(currentRandNum) {
//                case 0:
//                    setBlinkinColor(Variables.BlinkinColor.GREEN);
//                    break;
//                case 1:
//                    setBlinkinColor(Variables.BlinkinColor.YELLOW);
//                    break;
//                case 2:
//                    setBlinkinColor(Variables.BlinkinColor.BLUE);
//                    break;
//                case 3:
//                    setBlinkinColor(Variables.BlinkinColor.RED);
//                    break;
//                case 4:
//                    setBlinkinColor(Variables.BlinkinColor.ORANGE);
//                    break;
//                case 5:
//                    setBlinkinColor(Variables.BlinkinColor.PINK);
//                    break;
//                case 6:
//                    setBlinkinColor(Variables.BlinkinColor.RAINBOW);
//                    break;
//            }
//        }
        waitForStart();
        resetRuntime();
        rotateAngle(90);
        driveForDistance(0.5, Variables.Direction.FORWARD, 0.2);
        rotateAngle(-90);
        telemetry.clearAll();
//        telemetry.addLine("\"Oops\", accidentally created this.");

        while (opModeIsActive()) {
//            currentRandNum = intrng(0, 6);
//            switch(currentRandNum) {
//                case 0:
//                    setBlinkinColor(Variables.BlinkinColor.GREEN);
//                    break;
//                case 1:
//                    setBlinkinColor(Variables.BlinkinColor.YELLOW);
//                    break;
//                case 2:
//                    setBlinkinColor(Variables.BlinkinColor.BLUE);
//                    break;
//                case 3:
//                    setBlinkinColor(Variables.BlinkinColor.RED);
//                    break;
//                case 4:
//                    setBlinkinColor(Variables.BlinkinColor.ORANGE);
//                    break;
//                case 5:
//                    setBlinkinColor(Variables.BlinkinColor.PINK);
//                    break;
//                case 6:
//                    setBlinkinColor(Variables.BlinkinColor.RAINBOW);
//                    break;
//        }
//        rngdegnum = intrng(-180, 180);
        telemetry.addLine("Rotation degrees: " + rngdegnum);
        telemetry.update();
//            if(getRuntime() > 15.0) {
//                rngdegnum = intrng(-180,180);
////                telemetry.addLine("\"Oops\", accidentally created this.");
//                telemetry.addLine("Have your eyes melted yet?");
//                telemetry.update();
//            } else {
//                telemetry.update();
//            }
//        rotateToHeading(rngdegnum);
    }
    }
}