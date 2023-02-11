package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.globalTargetRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name ="BBMultiHLT", group = "A")
public class BBMultiHLT extends DriveMethods{
    OpenCvWebcam webcam;
    private String result;

    public void runOpMode() {

        DamienCVPipelineRB_BB pipeline = new DamienCVPipelineRB_BB();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(2500);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });



        globalTargetRotation = 0;

        initMotorsBlue();
        clawClamp();
        sleep(500);


        while(!isStarted()) {
            telemetry.addLine("result: " + pipeline.getCurrentResultsStr());
            telemetry.update();
            result = pipeline.getCurrentResultsStr();

        }
        waitForStart();

        GoToHeight(300);
        driveForDistance(0.06, Variables.Direction.FORWARD,0.4, globalTargetRotation);
        driveForDistance(0.65, Variables.Direction.LEFT,0.4, globalTargetRotation);
        driveForDistance(0.55, Variables.Direction.FORWARD,0.4, globalTargetRotation);
        driveForDistance(0.37, Variables.Direction.LEFT, 0.4, globalTargetRotation);
        goToHigh();
        sleep(100);
        driveForDistance(0.135, Variables.Direction.FORWARD,0.2, globalTargetRotation);
        sleep(100);
        clawRelease();
        sleep(100);
        driveForDistance(0.10, Variables.Direction.BACKWARD,0.4, globalTargetRotation);
        GoToHeight(1000);
        sleep(100);
        driveForDistance(0.35, Variables.Direction.RIGHT, 0.5, globalTargetRotation);
        driveForDistance(0.7, Variables.Direction.FORWARD, 0.5, globalTargetRotation);
        rotateWithBrake(-90);

        globalTargetRotation = -90;
        driveForDistance(.9, Variables.Direction.FORWARD, .5, globalTargetRotation);
        driveForDistance(.02, Variables.Direction.LEFT, .5, globalTargetRotation);
        GoToHeight(1200);
        sleep(100);
        driveForDistance(.4, Variables.Direction.FORWARD, .25, globalTargetRotation);
        sleep(100);
        GoToHeight(615);
        sleep(200);
        clawClamp();
        sleep(200);
        goToLow();
        driveForDistance(.32, Variables.Direction.BACKWARD, .45, globalTargetRotation);
        rotateAngle(-178);
        globalTargetRotation = -178;
        driveForDistance(.07, Variables.Direction.FORWARD, .5, globalTargetRotation);
        sleep(250);
        clawRelease();
        sleep(250);
        driveForDistance(.07, Variables.Direction.BACKWARD, .8, globalTargetRotation);
        sleep(150);
        rotateWithBrake(-90);
        globalTargetRotation = -90;
        GoToHeight(1000);
        sleep(100);
        driveForDistance(.4, Variables.Direction.FORWARD, .25, globalTargetRotation);
        GoToHeight(460);
        sleep(100);
        clawClamp();
        sleep(100);
        GoToHeight(1000);
        driveForDistance(.07, Variables.Direction.BACKWARD, .6, globalTargetRotation);
        goToCollect();
        rotateWithBrake(-180);
        globalTargetRotation = -180;
        driveForDistance(1.2, Variables.Direction.FORWARD, .4, globalTargetRotation);
        sleep(500);
        driveForDistance(.3, Variables.Direction.LEFT, 1, globalTargetRotation);
        clawRelease();
        sleep(1000);
        driveForDistance(1.2, Variables.Direction.BACKWARD, 1, globalTargetRotation);
        driveForDistance(1.3, Variables.Direction.BACKWARD, 1, globalTargetRotation);
        sleep(500);
        driveForDistance(.3, Variables.Direction.RIGHT, .6, globalTargetRotation);
        driveForDistance(.1, Variables.Direction.BACKWARD, .6, globalTargetRotation);
        //DOES NOT WORK YET

        switch(result){
            case "purple":
            // Your Where U Need to Be
                break;
            case "yellow":
                driveForDistance(0.7, Variables.Direction.BACKWARD, 0.45, globalTargetRotation);
                break;
            case "green":
                driveForDistance(1.3, Variables.Direction.BACKWARD, 0.45, globalTargetRotation);
                break;
        }


        while (opModeIsActive()) {

        }
    }
}
