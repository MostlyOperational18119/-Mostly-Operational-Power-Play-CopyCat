package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.globalTargetRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name ="BBMultiTLH", group = "A")
public class BBMultiTLH extends DriveMethods{
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
        driveForDistance(.05, Variables.Direction.FORWARD, .3, globalTargetRotation);
        rotateWithBrake(-90);
        globalTargetRotation = -90;
        driveForDistanceBrake(.82, Variables.Direction.FORWARD, .4, globalTargetRotation);
        sleep(100);
        clawRelease();
        GoToHeight(550);
        sleep(400);
        driveForDistanceBrake(.13, Variables.Direction.BACKWARD, .3, globalTargetRotation);
        rotateWithBrake(0);
        globalTargetRotation = 0;
        driveForDistanceBrake(1.34, Variables.Direction.FORWARD, .4, globalTargetRotation);
        driveForDistanceBrake(.1, Variables.Direction.LEFT, .4, globalTargetRotation);
        rotateWithBrake(-88);
        globalTargetRotation = -88;
        GoToHeight(1250);
        sleep(100);
        driveForDistanceBrake(.20, Variables.Direction.FORWARD, .25, globalTargetRotation);
        sleep(100);
        GoToHeight(615);
        sleep(200);
        clawClamp();
        sleep(200);
        GoToHeight(1600);
        driveForDistanceBrake(.4, Variables.Direction.BACKWARD, .45, globalTargetRotation);
        rotateAngle(-176);
        globalTargetRotation = -176;
        sleep(100);
        driveForDistanceBrake(.14, Variables.Direction.FORWARD, .3, globalTargetRotation);
        sleep(250);
        clawRelease();
        sleep(250);
        driveForDistanceBrake(.17, Variables.Direction.BACKWARD, .5, globalTargetRotation);
        sleep(150);
        rotateWithBrake(-90);
        globalTargetRotation = -90;
        driveForDistanceBrake(.42, Variables.Direction.FORWARD, .35, globalTargetRotation);
        GoToHeight(460);
        sleep(200);
        clawClamp();
        sleep(200);
        GoToHeight(1600);
        driveForDistanceBrake(1.76, Variables.Direction.BACKWARD, .5, globalTargetRotation);
        rotateWithBrake(-180);
        globalTargetRotation = -180;
        goToHigh();
        sleep(100);
        driveForDistanceBrake(0.125, Variables.Direction.FORWARD,0.25, globalTargetRotation);
        sleep(100);
        clawRelease();
        sleep(100);
        driveForDistanceBrake(0.14, Variables.Direction.BACKWARD,0.5, globalTargetRotation);
        goToLow();
        sleep(100);
        rotateWithBrake(-90);
        globalTargetRotation = -90;
        switch(result){
            case "purple":
                driveForDistanceBrake(1.45, Variables.Direction.FORWARD, .6, globalTargetRotation);
                goToDown();
                break;
            case "yellow":
                driveForDistanceBrake(.85, Variables.Direction.FORWARD, 0.6, globalTargetRotation);
                break;
            case "green":
                driveForDistanceBrake(.15, Variables.Direction.FORWARD, 0.6, globalTargetRotation);
                break;
        }


        while (opModeIsActive()) {

        }
    }
}
