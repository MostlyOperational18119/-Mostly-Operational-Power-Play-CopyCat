package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.globalTargetRotation;
import static org.firstinspires.ftc.teamcode.Variables.highHeight;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name ="BRMultistack", group = "A")
@Disabled
public class BRMultistack extends DriveMethods{
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
        driveForDistance(0.605, Variables.Direction.RIGHT,0.35, globalTargetRotation);
        driveForDistance(1.22, Variables.Direction.FORWARD,0.35, globalTargetRotation);
        driveForDistance(0.40, Variables.Direction.LEFT, 0.35, globalTargetRotation);
        goToHigh();
        driveForDistance(0.09, Variables.Direction.FORWARD,0.2, globalTargetRotation);
        sleep(500);
        GoToHeight(highHeight-10);
        clawRelease();
        sleep(200);
        driveForDistance(0.09, Variables.Direction.BACKWARD,0.35, globalTargetRotation);
        goToDown();
        sleep(500);
        rotateAngle(86);
        globalTargetRotation = 86;
        driveForDistance(.85, Variables.Direction.FORWARD, .45, globalTargetRotation);
        GoToHeight(1000);
        sleep(500);
        driveForDistance(.11, Variables.Direction.FORWARD, .40, globalTargetRotation);
        GoToHeight(600);
        sleep(200);
        clawClamp();
        sleep(200);
        GoToHeight(1200);
        driveForDistance(.98, Variables.Direction.BACKWARD, .45, globalTargetRotation);
//        driveForDistance(.81, Variables.Direction.BACKWARD, .35, globalTargetRotation);
        rotateAngle(0);
        globalTargetRotation = 0;
        goToHigh();
        driveForDistance(.17, Variables.Direction.FORWARD, .25, globalTargetRotation);
        sleep(250);
        GoToHeight(highHeight-10);
        sleep(250);
        clawRelease();
        sleep(300);
        driveForDistance(.10, Variables.Direction.BACKWARD, .35, globalTargetRotation);
        goToDown();


        switch(result){
            case "purple":
                driveForDistance(0.35, Variables.Direction.RIGHT, 0.5, globalTargetRotation);
                break;
            case "yellow":
                driveForDistance(0.35, Variables.Direction.LEFT, 0.5, globalTargetRotation);
                break;
            case "green":
                driveForDistance(.93, Variables.Direction.LEFT, 0.6, globalTargetRotation);
                break;
        }


        while (opModeIsActive()) {

        }
    }
}
