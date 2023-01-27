package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.globalTargetRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name ="LeftCenter", group = "A")
@Disabled
public class LeftCenterAuto extends DriveMethods{
    OpenCvWebcam webcam;
    private String result;

    public void runOpMode() {

        DamienCVPipelineBR_RR pipeline = new DamienCVPipelineBR_RR();
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
        driveForDistance(0.63, Variables.Direction.RIGHT,0.35, globalTargetRotation);
        driveForDistance(0.55, Variables.Direction.FORWARD,0.35, globalTargetRotation);
        driveForDistance(0.33, Variables.Direction.RIGHT, 0.35, globalTargetRotation);
        goToHigh();
        driveForDistance(0.07, Variables.Direction.FORWARD,0.2, globalTargetRotation);
        sleep(500);
        clawRelease();
        sleep(200);
        driveForDistance(0.135, Variables.Direction.BACKWARD,0.35, globalTargetRotation);
        goToDown();
        sleep(500);
        driveForDistance(0.35, Variables.Direction.LEFT, 0.35, globalTargetRotation);
        driveForDistance(0.6, Variables.Direction.FORWARD, 0.35, globalTargetRotation);
        switch(result){
            case "purple":
                //You're where you need to be!
                break;
            case "yellow":
                driveForDistance(0.7, Variables.Direction.LEFT, 0.35, globalTargetRotation);
                break;
            case "green":
                driveForDistance(1.37, Variables.Direction.LEFT, 0.35, globalTargetRotation);
                break;


        }


        while (opModeIsActive()) {

        }
    }
}
