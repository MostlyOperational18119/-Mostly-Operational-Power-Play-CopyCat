package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.globalTargetRotation;
import static org.firstinspires.ftc.teamcode.Variables.highHeight;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name ="RBMultistack", group = "A")
//@Disabled
public class RBMultistack extends DriveMethods{
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
        driveForDistance(0.625, Variables.Direction.LEFT,0.35, globalTargetRotation);
        driveForDistance(1.22, Variables.Direction.FORWARD,0.35, globalTargetRotation);
        driveForDistance(0.345, Variables.Direction.RIGHT, 0.35, globalTargetRotation);
        goToHigh();
        driveForDistance(0.16, Variables.Direction.FORWARD,0.2, globalTargetRotation);
        sleep(500);
        GoToHeight(highHeight-10);
        clawRelease();
        sleep(200);
        driveForDistance(0.1, Variables.Direction.BACKWARD,0.35, globalTargetRotation);
        goToDown();
        sleep(500);
        rotateAngle(-86);
        globalTargetRotation = -86;
        driveForDistance(.85, Variables.Direction.FORWARD, .35, globalTargetRotation);
        GoToHeight(1000);
        sleep(500);
        driveForDistance(.17, Variables.Direction.FORWARD, .30, globalTargetRotation);
        GoToHeight(615);
        sleep(250);
        clawClamp();
        sleep(250);
        GoToHeight(1200);
        driveForDistance(1.06, Variables.Direction.BACKWARD, .35, globalTargetRotation);
        rotateAngle(0);
        globalTargetRotation = 0;
        goToHigh();
        driveForDistance(.12, Variables.Direction.FORWARD, .2, globalTargetRotation);
        sleep(250);
        GoToHeight(highHeight-10);
        sleep(250);
        clawRelease();
        sleep(300);
        driveForDistance(.07, Variables.Direction.BACKWARD, .2, globalTargetRotation);
        goToDown();


        switch(result){
            case "purple":
                driveForDistance(0.95, Variables.Direction.RIGHT, 0.6, globalTargetRotation);

                break;
            case "yellow":
                driveForDistance(0.35, Variables.Direction.RIGHT, 0.35, globalTargetRotation);
                break;
            case "green":
                driveForDistance(0.35, Variables.Direction.LEFT, 0.35, globalTargetRotation);

                break;
        }


        while (opModeIsActive()) {

        }
    }
}
