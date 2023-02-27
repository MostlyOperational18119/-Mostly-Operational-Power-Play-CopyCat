package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.globalTargetRotation;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name ="BRMultiLMH", group = "A")
@Disabled
public class BRMultiLMH extends DriveMethods{
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
        driveForDistance(.06, Variables.Direction.LEFT, .45, globalTargetRotation);
        rotateWithBrake(90);
        globalTargetRotation = 90;
        driveForDistanceBrake(1.10, Variables.Direction.RIGHT,0.45, globalTargetRotation);
        goToLow();
        driveForDistanceBrake(0.16, Variables.Direction.FORWARD,0.35, globalTargetRotation);
        sleep(100);
        clawRelease();
        sleep(100);
        driveForDistanceBrake(0.13, Variables.Direction.BACKWARD,0.4, globalTargetRotation);
        driveForDistanceBrake(0.44, Variables.Direction.RIGHT,0.45, globalTargetRotation);
        driveForDistanceBrake(0.57, Variables.Direction.FORWARD,0.45, globalTargetRotation);
        GoToHeight(1450);
        sleep(100);
        driveForDistanceBrake(.10, Variables.Direction.FORWARD, .20, globalTargetRotation);
        sleep(200);
        GoToHeight(600);
        sleep(200);
        clawClamp();
        telemetry.addLine("hi");
        telemetry.update();
        sleep(300);
        GoToHeight(1600);
        driveForDistanceBrake(1.76, Variables.Direction.BACKWARD, .45, globalTargetRotation);
        rotateAngle(180);
        globalTargetRotation = 180;
//        sleep(100);
        goToHigh();
        sleep(200);
        driveForDistanceBrake(.14, Variables.Direction.FORWARD, .3, globalTargetRotation);
        sleep(350);
        clawRelease();
        sleep(250);
        driveForDistanceBrake(.13, Variables.Direction.BACKWARD, .5, globalTargetRotation);
//        sleep(150);
        rotateWithBrake(90);
        globalTargetRotation = 90;
        GoToHeight(1250);
        driveForDistanceBrake(1.60, Variables.Direction.FORWARD, .5, globalTargetRotation);
        GoToHeight(1650);
        sleep(100);
        driveForDistance(0.1, Variables.Direction.FORWARD, .25, globalTargetRotation);
        GoToHeight(425);
        sleep(200);
        clawClamp();
        sleep(300);
        GoToHeight(1600);
        driveForDistanceBrake(1.1, Variables.Direction.BACKWARD, .55, globalTargetRotation);
        rotateWithBrake(180);
        globalTargetRotation = 180;
        goToMid();
        sleep(100);
        driveForDistanceBrake(0.135, Variables.Direction.FORWARD,0.3, globalTargetRotation);
        sleep(200);
        clawRelease();
        sleep(100);
        driveForDistanceBrake(0.14, Variables.Direction.BACKWARD,0.5, globalTargetRotation);
        goToLow();
        sleep(100);
        rotateWithBrake(90);
        globalTargetRotation = 90;
        switch(result){
            case "purple":
                driveForDistanceBrake(.2, Variables.Direction.BACKWARD, 0.6, globalTargetRotation);
                break;
            case "yellow":
                driveForDistanceBrake(.2, Variables.Direction.FORWARD, 0.6, globalTargetRotation);
                break;
            case "green":
                driveForDistanceBrake(.8, Variables.Direction.FORWARD, .6, globalTargetRotation);
                break;

        }
        goToDown();




//        rotateWithBrake(90);
//        globalTargetRotation = 90;
//        GoToHeight(1250);
//        sleep(100);
//        driveForDistanceBrake(0.20, Variables.Direction.FORWARD,0.25, globalTargetRotation);
//        sleep(100);
//        GoToHeight(615);
//        sleep(200);
//        clawClamp();
//        sleep(200);
//        goToLow();
//        driveForDistanceBrake(0.82, Variables.Direction.BACKWARD,0.4, globalTargetRotation);
//        rotateWithBrake(180);
//        globalTargetRotation = 180;
//        goToMid();
//        driveForDistanceBrake(0.08, Variables.Direction.FORWARD,0.4, globalTargetRotation);
//        sleep(250);
//        clawRelease();
//        sleep(250);
//        driveForDistanceBrake(0.08, Variables.Direction.BACKWARD,0.4, globalTargetRotation);
//        rotateWithBrake(90);
//        globalTargetRotation = 90;
//        driveForDistanceBrake(0.82, Variables.Direction.FORWARD,0.4, globalTargetRotation);
//        sleep(100);
//        GoToHeight(450);
//        sleep(200);
//        clawClamp();
//        sleep(200);
//        goToLow();
//        driveForDistanceBrake(1.43, Variables.Direction.BACKWARD,0.4, globalTargetRotation);
//        rotateWithBrake(180);
//        globalTargetRotation = 180;
//        goToHigh();
//        driveForDistanceBrake(0.08, Variables.Direction.FORWARD,0.4, globalTargetRotation);
//        sleep(250);
//        clawRelease();
//        sleep(250);
//        driveForDistanceBrake(0.08, Variables.Direction.BACKWARD,0.4, globalTargetRotation);
//        rotateWithBrake(90);
//        globalTargetRotation = 90;
//        driveForDistanceBrake(0.2, Variables.Direction.FORWARD,0.6, globalTargetRotation);
//
//
//
//
//        driveForDistanceBrake(0.40, Variables.Direction.LEFT, 0.4, globalTargetRotation);
//        goToHigh();
//        sleep(100);
//        driveForDistanceBrake(0.165, Variables.Direction.FORWARD,0.35, globalTargetRotation);
//        sleep(100);
//        clawRelease();
//        sleep(100);
//        driveForDistanceBrake(0.16, Variables.Direction.BACKWARD,0.4, globalTargetRotation);
//        GoToHeight(1000);
//        sleep(100);
//        driveForDistanceBrake(0.35, Variables.Direction.RIGHT, 0.5, globalTargetRotation);
//        driveForDistanceBrake(0.65, Variables.Direction.FORWARD, 0.5, globalTargetRotation);
//        driveForDistance(.85, Variables.Direction.FORWARD, .5, globalTargetRotation);
//        sleep(50);
//        driveForDistanceBrake(.05, Variables.Direction.LEFT, .5, globalTargetRotation);
//        GoToHeight(1200);
//        sleep(100);
//        driveForDistanceBrake(.45, Variables.Direction.FORWARD, .25, globalTargetRotation);
//        sleep(100);
//        GoToHeight(615);
//        sleep(200);
//        clawClamp();
//        sleep(200);
//        goToLow();
//        driveForDistanceBrake(.4, Variables.Direction.BACKWARD, .45, globalTargetRotation);
//        rotateAngle(-178);
//        globalTargetRotation = -178;
//        sleep(100);
//        driveForDistanceBrake(.14, Variables.Direction.FORWARD, .3, globalTargetRotation);
//        sleep(250);
//        clawRelease();
//        sleep(250);
//        driveForDistanceBrake(.14, Variables.Direction.BACKWARD, .5, globalTargetRotation);
//        sleep(150);
//        rotateWithBrake(-90);
//        globalTargetRotation = -90;
//        sleep(100);
//        driveForDistanceBrake(.42, Variables.Direction.FORWARD, .35, globalTargetRotation);
//        GoToHeight(460);
//        sleep(100);
//        clawClamp();
//        sleep(100);
//        GoToHeight(800);
//        driveForDistanceBrake(.1, Variables.Direction.BACKWARD, .4, globalTargetRotation);
//        goToCollect();
//        rotateWithBrake(-180);
//        globalTargetRotation = -180;
//        driveForDistanceBrake(1.2, Variables.Direction.FORWARD, .6, globalTargetRotation);
//        driveForDistanceBrake(.3, Variables.Direction.LEFT, .35, globalTargetRotation);
//        clawRelease();

        //DOES NOT WORK YET

//        switch(result){
//            case "purple":
//            // Your Where U Need to Be
//                break;
//            case "yellow":
//                driveForDistanceBrake(0.7, Variables.Direction.BACKWARD, 0.45, globalTargetRotation);
//                break;
//            case "green":
//                driveForDistanceBrake(1.3, Variables.Direction.BACKWARD, 0.45, globalTargetRotation);
//                break;
//        }


        while (opModeIsActive()) {

        }
    }
}
