package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.collectHeight;
import static org.firstinspires.ftc.teamcode.Variables.globalTargetRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name ="RedOppSide", group = "A")
@Disabled
public class RedOppSide extends DriveMethods{
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

        GoToHeight(collectHeight);
        GoToHeight(300);
        driveForDistance(0.1, Variables.Direction.FORWARD,0.35, globalTargetRotation);
        driveForDistance(0.65, Variables.Direction.LEFT,0.35, globalTargetRotation);
        driveForDistance(1.22, Variables.Direction.FORWARD,0.35, globalTargetRotation);
        driveForDistance(0.38, Variables.Direction.RIGHT, 0.35, globalTargetRotation);
        goToHigh();
        driveForDistance(0.13, Variables.Direction.FORWARD,0.2, globalTargetRotation);
        sleep(500);
        clawRelease();
        sleep(200);
        driveForDistance(0.16, Variables.Direction.BACKWARD,0.35, globalTargetRotation);
        goToLow();
        rotateAngle(-90);
        driveForDistance(.75, Variables.Direction.FORWARD, .35, globalTargetRotation);
        goToFifth();
        clawClamp();
        goToLow();
        driveForDistance(.75, Variables.Direction.BACKWARD, .35, globalTargetRotation);
        rotateAngle(90);
        goToDown();
        rotateToHeading(90, 0.2);
        driveForDistance(.2, Variables.Direction.FORWARD, .35, globalTargetRotation);



        //Divergence point
        switch(result){
            case "purple":

                break;
            case "yellow":

                break;
            case "green":

                break;

        }



        while (opModeIsActive()) {

        }
    }
}