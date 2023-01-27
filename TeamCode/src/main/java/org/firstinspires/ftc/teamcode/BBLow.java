package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static org.firstinspires.ftc.teamcode.Variables.*;

@Autonomous(name ="BBLow", group = "A")
@Disabled
public class BBLow extends DriveMethods {
    public void runOpMode() {
        globalTargetRotation = 0;
        initMotorsBlue();


        clawClamp();
        waitForStart();

        GoToHeight(1950);
        driveForDistance(0.05, Direction.FORWARD, 0.35, globalTargetRotation);
        driveForDistance(0.24, Direction.LEFT, 0.35, globalTargetRotation);
        driveForDistance(0.17, Direction.FORWARD, 0.35, globalTargetRotation);
        sleep(500);
        clawRelease();
        sleep(1000);
        driveForDistance(0.3, Direction.BACKWARD, 0.5, globalTargetRotation);

        GoToHeight(0);

        driveForDistance(1.4, Direction.RIGHT, 0.5, globalTargetRotation);

        while(opModeIsActive()) {

        }
    }



}
