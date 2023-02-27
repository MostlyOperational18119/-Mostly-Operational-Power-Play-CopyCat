package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.globalTargetRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name ="Circuit 1", group = "A")
public class Circuit1Auto extends DriveMethods{
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



//        PipeConeTracker pipeConeTracker = new PipeConeTracker("one", "blue");
//        webcam.setPipeline(pipeConeTracker);
        GoToHeight(350);
        driveForDistanceBrake(0.1, Variables.Direction.FORWARD,0.45,globalTargetRotation);
        driveForDistanceBrake(0.54, Variables.Direction.LEFT,0.5,globalTargetRotation);
        driveForDistance(0.26, Variables.Direction.FORWARD,0.3,globalTargetRotation);
        goToDown();
        sleep(100);
        clawRelease();
        sleep(350);
        GoToHeight(1000);
        driveForDistanceBrake(0.1, Variables.Direction.BACKWARD,0.45,globalTargetRotation);
        driveForDistanceBrake(0.33, Variables.Direction.LEFT,0.45,globalTargetRotation);
        driveForDistanceBrake(1.20, Variables.Direction.FORWARD,0.45,globalTargetRotation);
        driveForDistanceBrake(0.25, Variables.Direction.RIGHT,0.45,globalTargetRotation);
//        GoToHeight(0);
        rotateWithBrake(90);
        globalTargetRotation = 90;
//        sleep(500);
//        alignToConeStack(webcam,globalTargetRotation,"blue");
        GoToHeight(1450);
//        sleep(100);
        driveForDistance(.37, Variables.Direction.FORWARD, .3, globalTargetRotation);
        sleep(100);
        GoToHeight(615);
        sleep(200);
        clawClamp();
        sleep(200);
        GoToHeight(1600);
        driveForDistanceBrake(.15, Variables.Direction.BACKWARD, .45, globalTargetRotation);
        GoToHeight(300);
        rotateWithBrake(0);
        globalTargetRotation = 0;
//        driveForDistanceBrake(0.05, Variables.Direction.LEFT,0.45,globalTargetRotation);
        driveForDistanceBrake(0.4, Variables.Direction.FORWARD,0.65,globalTargetRotation);
        driveForDistanceBrake(0.15, Variables.Direction.LEFT,0.45,globalTargetRotation);
        driveForDistanceBrake(0.18, Variables.Direction.FORWARD,0.65,globalTargetRotation);
        sleep(100);
        clawRelease();
        GoToHeight(550);
        sleep(300);
        driveForDistanceBrake(0.2, Variables.Direction.BACKWARD,0.65,globalTargetRotation);
        driveForDistanceBrake(0.15, Variables.Direction.RIGHT,0.45,globalTargetRotation);
        driveForDistanceBrake(0.465, Variables.Direction.BACKWARD,0.65,globalTargetRotation);
        driveForDistanceBrake(0.1, Variables.Direction.RIGHT,0.45,globalTargetRotation);
        rotateWithBrake(88);
        globalTargetRotation = 88;
//        GoToHeight(200);
//        sleep(500);
//        alignToConeStack(webcam,globalTargetRotation,"blue");
        GoToHeight(1250);
        sleep(100);
        driveForDistance(.22, Variables.Direction.FORWARD, 0.3, globalTargetRotation);
        sleep(100);
        GoToHeight(540);
        sleep(200);
        clawClamp();
        sleep(200);
        GoToHeight(1600);
        driveForDistanceBrake(0.36, Variables.Direction.BACKWARD, .55, globalTargetRotation);
        rotateWithBrake(180);
        globalTargetRotation = 180;
        goToLow();
        sleep(100);
        driveForDistanceBrake(0.145, Variables.Direction.FORWARD,0.3, globalTargetRotation);
        sleep(200);
        clawRelease();
        sleep(100);
        driveForDistanceBrake(0.14, Variables.Direction.BACKWARD,0.45, globalTargetRotation);
        goToDown();
        sleep(100);
//        rotateWithBrake(-88);
//        globalTargetRotation = -88;
        switch(result){
            case "green":
                rotateWithBrake(88);
                globalTargetRotation= 88;
                driveForDistanceBrake(.2, Variables.Direction.FORWARD, 0.45, globalTargetRotation);
                break;
            case "yellow":
                driveForDistanceBrake(.3, Variables.Direction.LEFT, 0.85, globalTargetRotation);
                rotateWithBrake(180);
                globalTargetRotation= 180;
                break;
            case "purple":
                driveForDistanceBrake(.85, Variables.Direction.LEFT, 0.85, globalTargetRotation);
                break;

        }





//        goToDown();
//        switch(result){
//            case "purple":
//                driveForDistanceBrake(1.45, Variables.Direction.FORWARD, .65, globalTargetRotation);
//                break;
//            case "yellow":
//                driveForDistanceBrake(.85, Variables.Direction.FORWARD, 0.65, globalTargetRotation);
//                break;
//            case "green":
//                driveForDistanceBrake(.15, Variables.Direction.FORWARD, 0.65, globalTargetRotation);
//                break;
//        }


        while (opModeIsActive()) {

        }
    }
}
