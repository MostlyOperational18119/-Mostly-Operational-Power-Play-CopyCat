package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.globalTargetRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name ="BBLLM", group = "A")
public class BBLLM extends DriveMethods{
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

        GoToHeight(400);
        driveForDistance(0.06, Variables.Direction.FORWARD,0.45, globalTargetRotation);
        driveForDistance(.06, Variables.Direction.RIGHT, .45, globalTargetRotation);
        rotateWithBrake(-87);
        globalTargetRotation = -87;
        driveForDistanceBrake(1.10, Variables.Direction.LEFT,0.7, globalTargetRotation);
        goToLow();
        driveForDistanceBrake(0.16, Variables.Direction.FORWARD,0.35, globalTargetRotation);
        sleep(100);
        clawRelease();
        sleep(100);
        driveForDistanceBrake(0.13, Variables.Direction.BACKWARD,0.4, globalTargetRotation);
        driveForDistanceBrake(0.37, Variables.Direction.LEFT,0.5, globalTargetRotation);
        driveForDistanceBrake(0.57, Variables.Direction.FORWARD,0.5, globalTargetRotation);
        GoToHeight(1650);
        sleep(100);
        driveForDistance(.115, Variables.Direction.FORWARD, .25, globalTargetRotation);
        sleep(200);
        GoToHeight(500);
        sleep(200);
        clawClamp();
        sleep(200);
        sleep(300);
        GoToHeight(1600);
        driveForDistanceBrake(1.035, Variables.Direction.BACKWARD, .7, globalTargetRotation);
//        driveForDistanceBrake(.1, Variables.Direction.LEFT, .4, globalTargetRotation);
        rotateWithBrake(-0);
        globalTargetRotation = 0;
//        sleep(100);
        goToLow();
        sleep(200);
        driveForDistanceBrake(.15, Variables.Direction.FORWARD, .3, globalTargetRotation);
        sleep(300);
//        driveForDistanceBrake(.03, Variables.Direction.BACKWARD, .2,globalTargetRotation);
//        sleep(200);
        clawRelease();
        sleep(250);
        driveForDistanceBrake(.15, Variables.Direction.BACKWARD, .5, globalTargetRotation);
        GoToHeight(1650);
        rotateWithBrake(-89.5);
        globalTargetRotation = -89.5;
        driveForDistanceBrake(1.45, Variables.Direction.FORWARD, .8, globalTargetRotation);
        GoToHeight(1650);
        sleep(100);
        driveForDistance(0.06, Variables.Direction.FORWARD, .25, globalTargetRotation);
        GoToHeight(425);
        sleep(200);
        clawClamp();
        sleep(300);
        GoToHeight(1600);
        driveForDistanceBrake(1.035, Variables.Direction.BACKWARD, .7, globalTargetRotation);
        rotateWithBrake(-180);
        globalTargetRotation = -180;
        goToMid();
        sleep(100);
        driveForDistanceBrake(0.135, Variables.Direction.FORWARD,0.3, globalTargetRotation);
        sleep(200);
        clawRelease();
        sleep(100);
        driveForDistanceBrake(0.14, Variables.Direction.BACKWARD,0.5, globalTargetRotation);
        goToLow();
        sleep(100);
        rotateWithBrake(-90);
        globalTargetRotation = -90;
        goToDown();
        switch(result){
            case "purple":
                driveForDistanceBrake(.8, Variables.Direction.FORWARD, .85, globalTargetRotation);
                break;
            case "yellow":
                driveForDistanceBrake(.2, Variables.Direction.FORWARD, 0.85, globalTargetRotation);
                break;
            case "green":
                driveForDistanceBrake(.2, Variables.Direction.BACKWARD, 0.85, globalTargetRotation);
                break;

        }




        while (opModeIsActive()) {

        }
    }
}
