package TestPrograms;

import static org.firstinspires.ftc.teamcode.Variables.motorSlide;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.DriveMethods;
import org.firstinspires.ftc.teamcode.PipePoleTracker;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="CVTestAuto", group="A")
public class CVAutoTest extends DriveMethods {

    @Override
    public void runOpMode() {

        initMotorsBlue();
        calibrateNavXIMU();
        motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);





        /**
         * Camera initialization stuff below
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        PipePoleTracker pipePoleTracker = new PipePoleTracker("one");
        camera.setPipeline(pipePoleTracker);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        /**
         * Camera initialization stuff above
         */


        waitForStart();

        while(opModeIsActive()){


            if(gamepad2.a){
                alignToPole(camera);


                telemetry.addLine("Back in regular Teleop world!");
                telemetry.update();

            }
        }



    }
}
