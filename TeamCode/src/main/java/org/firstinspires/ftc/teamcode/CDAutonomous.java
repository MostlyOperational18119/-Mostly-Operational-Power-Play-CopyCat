package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "CDAutonomous", group = "A")
public class CDAutonomous extends DriveMethods {
    public void runOpmode() {
        initMotorsBlue();
        int num = 1;

        if (num == 1) {
            driveForDistance(.65, Variables.Direction.RIGHT, .35, 0);
            driveForDistance(1.22, Variables.Direction.FORWARD, .35, 0);
        }

        if (num == 2) {
            driveForDistance(.65, Variables.Direction.RIGHT, .35, 0);
            driveForDistance(1.22, Variables.Direction.FORWARD, .35, 0);
            driveForDistance(.65, Variables.Direction.LEFT, .35, 0);
        }

        if (num == 3) {
            driveForDistance(.65, Variables.Direction.LEFT, .35, 0);
            driveForDistance(1.22, Variables.Direction.FORWARD, .35, 0);
        }
    }
}
