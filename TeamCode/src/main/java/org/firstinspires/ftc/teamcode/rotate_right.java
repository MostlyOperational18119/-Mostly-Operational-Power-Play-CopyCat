package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name ="rotate", group = "A")
@Disabled
public class rotate_right extends DriveMethods{
    public void runOpMode() {
        initMotorsBlue();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("You are in teh loop!");
            telemetry.update();
        }
    }
}

