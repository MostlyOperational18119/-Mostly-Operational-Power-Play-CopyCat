package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import static org.firstinspires.ftc.teamcode.Variables.*;

@Autonomous(name ="RBLow", group = "A")
@Disabled
public class RBLow extends DriveMethods {
    public void runOpMode() {
        initMotorsBlue();

        clawClamp();
        waitForStart();

        GoToHeight(1950);
        driveForDistance(0.05, Direction.FORWARD, 0.3);
        driveForDistance(0.25, Direction.LEFT, 0.3);
        driveForDistance(0.17, Direction.FORWARD, 0.3);
        sleep(500);
        clawRelease();
        sleep(1000);
        driveForDistance(0.3, Direction.BACKWARD, 0.5);

        GoToHeight(0);

        driveForDistance(0.5, Direction.LEFT, 0.5);

        while(opModeIsActive()) {

        }
    }



}
