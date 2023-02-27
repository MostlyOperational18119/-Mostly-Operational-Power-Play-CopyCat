package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.globalTargetRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name ="RRMultiHLT", group = "A")
@Disabled
public class RRMultiHLT extends DriveMethods{
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
        driveForDistance(0.1, Variables.Direction.FORWARD,0.35, globalTargetRotation);
        driveForDistance(0.65, Variables.Direction.LEFT,0.35, globalTargetRotation);
        driveForDistance(0.63, Variables.Direction.FORWARD,0.35, globalTargetRotation);
        driveForDistance(0.34, Variables.Direction.LEFT, 0.35, globalTargetRotation);
        goToHigh();
        driveForDistance(0.07, Variables.Direction.FORWARD,0.2, globalTargetRotation);
        sleep(500);
        clawRelease();
        sleep(200);
        driveForDistance(0.135, Variables.Direction.BACKWARD,0.35, globalTargetRotation);
        goToDown();
        sleep(500);
        driveForDistance(0.35, Variables.Direction.RIGHT, 0.35, globalTargetRotation);
        driveForDistance(0.6, Variables.Direction.FORWARD, 0.35, globalTargetRotation);
        rotateWithBrake(-87);
        globalTargetRotation = -87;
        driveForDistance(.93, Variables.Direction.FORWARD, .45, globalTargetRotation);
        GoToHeight(1000);
        sleep(500);
        driveForDistance(.17, Variables.Direction.FORWARD, .45, globalTargetRotation);
        GoToHeight(615);
        sleep(250);
        clawClamp();
        sleep(250);
        goToLow();
        driveForDistance(.5, Variables.Direction.BACKWARD, .45, globalTargetRotation);
        rotateAngle(-178);
        globalTargetRotation = -178;
        driveForDistance(.07, Variables.Direction.FORWARD, .6, globalTargetRotation);
        sleep(250);
        clawRelease();
        sleep(250);
        driveForDistance(.07, Variables.Direction.BACKWARD, .6, globalTargetRotation);
        sleep(150);
        rotateWithBrake(-87);
        globalTargetRotation = -87;
        driveForDistance(.5, Variables.Direction.FORWARD, .6, globalTargetRotation);
        GoToHeight(615);
        clawClamp();
        sleep(100);
        goToLow();
        driveForDistance(.1, Variables.Direction.BACKWARD, .6, globalTargetRotation);
        rotateAngle(-178);
        globalTargetRotation = -178;
        driveForDistance(1.5, Variables.Direction.FORWARD, 1, globalTargetRotation);
        goToCollect();
        driveForDistance(.3, Variables.Direction.LEFT, 1, globalTargetRotation);
        clawRelease();
        driveForDistance(1.3, Variables.Direction.BACKWARD, 1, globalTargetRotation);
        driveForDistance(.3, Variables.Direction.RIGHT, .6, globalTargetRotation);
        driveForDistance(.2, Variables.Direction.BACKWARD, .6, globalTargetRotation);
        rotateAngle(-87);
        globalTargetRotation = -87;
        //DOES NOT WORK YET

        switch(result){
            case "green":
                //You're where you need to be!
                break;
            case "yellow":
                driveForDistance(0.7, Variables.Direction.RIGHT, 0.35, globalTargetRotation);
                break;
            case "purple":
                driveForDistance(1.37, Variables.Direction.RIGHT, 0.35, globalTargetRotation);
                break;


        }


        while (opModeIsActive()) {

        }
    }
}