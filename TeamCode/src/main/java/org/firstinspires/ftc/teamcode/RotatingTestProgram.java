package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.globalTargetRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Rotate test", group = "A")
public class RotatingTestProgram extends DriveMethods{
    @Override
    public void runOpMode() {
        globalTargetRotation = 0;
        initMotorsBlue();
        clawClamp();
        sleep(500);

        waitForStart();

        rotateAngle(90);
        driveForDistance(0.1, Variables.Direction.FORWARD,0.35, globalTargetRotation);
    }
}
