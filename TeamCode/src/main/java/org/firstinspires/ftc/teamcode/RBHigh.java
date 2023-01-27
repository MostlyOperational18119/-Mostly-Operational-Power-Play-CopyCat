package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.globalTargetRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name ="RBHigh", group = "A")
@Disabled
public class RBHigh extends DriveMethods{
    public void runOpMode() {
        globalTargetRotation = 0;
        initMotorsBlue();

        clawClamp();
        sleep(500);
        waitForStart();

        GoToHeight(300);

        driveForDistance(0.1, Variables.Direction.FORWARD,0.35, globalTargetRotation);
        driveForDistance(0.65, Variables.Direction.LEFT,0.35, globalTargetRotation);
        driveForDistance(1.22, Variables.Direction.FORWARD,0.35, globalTargetRotation);
        driveForDistance(0.35, Variables.Direction.RIGHT, 0.35, globalTargetRotation);
        goToHigh();
        driveForDistance(0.17, Variables.Direction.FORWARD,0.2, globalTargetRotation);
        sleep(500);
        clawRelease();
        sleep(200);
        driveForDistance(0.17, Variables.Direction.BACKWARD,0.35, globalTargetRotation);
        goToDown();
        sleep(500);
        driveForDistance(0.35, Variables.Direction.LEFT, 0.35, globalTargetRotation);
        driveForDistance(1.22, Variables.Direction.BACKWARD,0.35, globalTargetRotation);
        driveForDistance(.2, Variables.Direction.LEFT,0.35, globalTargetRotation);

        while (opModeIsActive()) {

        }
    }
}