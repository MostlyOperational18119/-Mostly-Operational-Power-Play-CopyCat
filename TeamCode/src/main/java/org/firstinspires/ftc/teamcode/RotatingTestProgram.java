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
@Disabled
public class RotatingTestProgram extends DriveMethods{
    @Override
    public void runOpMode() {
        globalTargetRotation = 0;
        calibrateNavXIMU();
        initMotorsBlue();
//        clawClamp();
//        sleep(500);

        waitForStart();

//        driveForDistance(1.3, Variables.Direction.FORWARD, .35,globalTargetRotation);
//        rotateAngle(90);
//        globalTargetRotation = 90;
//        driveForDistance(1.3, Variables.Direction.FORWARD, .35,globalTargetRotation);
//        rotateAngle(180);
//        globalTargetRotation = 180;
//        driveForDistance(1.3, Variables.Direction.FORWARD, .35,globalTargetRotation);
//        rotateAngle(270);
//        globalTargetRotation = 270;
//        driveForDistance(1.3, Variables.Direction.FORWARD, .35,globalTargetRotation);
//        rotateAngle(0);
//        globalTargetRotation = 0;
        telemetry.addLine("Cumalative Z: " + getCumulativeZ());
        telemetry.addLine("Current Z: " + getCurrentZ());
        telemetry.update();
    }
}
