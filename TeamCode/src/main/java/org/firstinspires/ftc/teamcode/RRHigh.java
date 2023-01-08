package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name ="RRHigh", group = "A")
@Disabled
public class RRHigh extends DriveMethods{
    public void runOpMode() {
        initMotorsBlue();

        clawClamp();
        sleep(500);
        waitForStart();

        GoToHeight(300);

        driveForDistance(0.1, Variables.Direction.FORWARD,0.35);
        driveForDistance(0.65, Variables.Direction.RIGHT,0.35);
        driveForDistance(1.22, Variables.Direction.FORWARD,0.35);
        driveForDistance(0.45, Variables.Direction.LEFT, 0.35);
        goToHigh();
        driveForDistance(0.135, Variables.Direction.FORWARD,0.2);
        sleep(500);
        clawRelease();
        sleep(200);
        driveForDistance(0.135, Variables.Direction.BACKWARD,0.35);
        goToDown();
        sleep(500);
        driveForDistance(0.35, Variables.Direction.RIGHT, 0.35);
        driveForDistance(1.22, Variables.Direction.BACKWARD,0.35);
        driveForDistance(1.5, Variables.Direction.LEFT,0.35);

        while (opModeIsActive()) {

        }
    }
}

