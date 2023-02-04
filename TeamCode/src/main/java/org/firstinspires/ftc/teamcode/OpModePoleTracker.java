package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxBL_X;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxBL_Y;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxHeight;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getBoxWidth;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getCenterX;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getHighestX;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getHighestY;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLargestObjectWidth;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLargestSize;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel1Assigment;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel2Assigment;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevel2Capable;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLevelString;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLowestX;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getLowestY;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getMinRectHeight;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getMinRectWidth;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getPercentColor;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getRectHeight;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getRectWidth;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getXResolution;
import static org.firstinspires.ftc.teamcode.PipePoleTracker.getYResolution;

import static org.firstinspires.ftc.teamcode.Variables.*;
import static org.firstinspires.ftc.teamcode.Variables.Direction.BACKWARD;
import static org.firstinspires.ftc.teamcode.Variables.Direction;
import static org.firstinspires.ftc.teamcode.Variables.Direction.FORWARD;
import static org.firstinspires.ftc.teamcode.Variables.Direction.LEFT;
import static org.firstinspires.ftc.teamcode.Variables.Direction.RIGHT;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="PoleTracker", group="A")
//@Disabled
public class OpModePoleTracker extends DriveMethods {
    String level = "one";
    int levelCounter = 1;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;



    //The unit here is pixels
    int targetX;
    int targetWidth;
    double errorX;
    int errorWidth;
    double currentWidth;
    double dividerX = 300;
    double alignPowerAddedX;
    double alignPowerAddedWidth;
    int slidePosition = 0;
    int targetHeight = 0;
    double leftX;
    double leftY;
    double rightX;


    //The unit here is boxes


    @Override
    public void runOpMode() {

        initMotorsBlue();
        motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        pattern = RevBlinkinLedDriver.BlinkinPattern.CONFETTI;
        blinkinLedDriver.setPattern(pattern);


        calibrateNavXIMU();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


        PipePoleTracker pipePoleTracker = new PipePoleTracker(level);
        camera.setPipeline(pipePoleTracker);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //Adjust this to reduce load?????
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);


            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
        waitForStart();

        targetX = 225; //<-- this SHOULD be the resolution at level1 (check-able)
        targetWidth = 15;
        level1Aligned = false;
        level2Aligned = false;
        level3Aligned = false;
//        isIMURecorded = false;
        visionAutoActivated = false;

//        GoToHeight(collectHeight);
        slidePosition = motorSlide.getCurrentPosition();


        while (opModeIsActive()) {

            leftY = -gamepad1.left_stick_y;
            leftX = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;

            motorFL.setPower((leftY + leftX + rightX) / 2);
            motorBL.setPower((leftY - leftX + rightX) / 2);
            motorFR.setPower((leftY - leftX - rightX) / 2);
            motorBR.setPower((leftY + leftX - rightX) / 2);

            if (gamepad1.dpad_up) {
                goToCollect();
            }
            if (gamepad1.dpad_down) {
                goToDown();
            }
            if (gamepad1.right_bumper) {
                clawClamp();
            }
            if (gamepad1.left_bumper) {
                clawRelease();
            }

            if (level2Capable == false && visionAutoActivated == false) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.RED;
                blinkinLedDriver.setPattern(pattern);
            }
            if (level2Capable == true && visionAutoActivated == false) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                blinkinLedDriver.setPattern(pattern);
            }
            if (visionAutoActivated && levelCounter == 1 || levelCounter == 2) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                blinkinLedDriver.setPattern(pattern);
            }

            if (visionAutoActivated && levelCounter == 3) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
                blinkinLedDriver.setPattern(pattern);
            }

            //Button triggers
            if (gamepad2.a && getLevel2Capable()) {
//                levelCounter = 2;
                visionAutoActivated = true;
            }


            if (gamepad2.x) {
                levelCounter = 1;
                level1Aligned = false;
                level2Aligned = false;
                level3Aligned = false;
                visionAutoActivated = false;
                stopMotors();
                motorSlide.setPower(0);
            }

            //this means we are no longer looking at our object of choice
            if (levelCounter != 3 && getLargestSize() == 0) {
                levelCounter = 1;
                level1Aligned = false;
                level2Aligned = false;
                level3Aligned = false;
                visionAutoActivated = false;
//                stopMotors();
//                motorSlide.setPower(0);
            }

            errorX = targetX - getCenterX();
            errorWidth = targetWidth - getLargestObjectWidth();

            dividerX = 300;


            if (visionAutoActivated) {

                alignPowerAddedX = errorX / dividerX;

                alignPowerAddedWidth = (double) errorWidth / 45;


                if (Math.abs(alignPowerAddedX) > 0.14) {
                    alignPowerAddedX = (errorX / (Math.abs(errorX))) * 0.14;
                }

                if (levelCounter == 1 && Math.abs(errorX) < 32) {//TODO will need to add distance condition
                    level1Aligned = true;
                    imuHeading = getCumulativeZ() + 1.5;
                    levelCounter = 2;
                    telemetry.addLine("level1 complete!");
                    telemetry.addLine("IMU Heading: " + imuHeading);
                    telemetry.addLine("errorX: " + errorX);
                    telemetry.addLine("errorX divide thingy: " + (errorX / (Math.abs(errorX))));

                    telemetry.update();
                    stopMotors();
                    //Robot is in front of pole well enough, entering level2...
                }

                if (levelCounter == 1 && level1Aligned == false) {

                    targetHeading = getCumulativeZ() + errorX*(0.04559197) + (0.007277*errorX) - 1; //The 0.045591... constant is derived from the width of camera view (angle) divided by the wide of the frame (pixels) to get degrees/pixel
                    telemetry.addLine("Target heading: " + targetHeading);
                    telemetry.addLine("Error heading: " + (errorX*(0.04559197)));
                    telemetry.addLine("Actual heading: " + getCumulativeZ());

                            rotateSmallWithBrake(targetHeading);

//                    }else {
//                        motorFL.setPower(-alignPowerAddedX);
//                        motorBL.setPower(-alignPowerAddedX);
//                        motorFR.setPower(alignPowerAddedX);
//                        motorBR.setPower(alignPowerAddedX);
//                    }

                }

                //Level2 below (untested at the moment - 1/17/23)
                if (levelCounter == 2 && getLevel2Assigment() == true) {
                    currentWidth = getLargestObjectWidth();                                                             //5.2 is an error adjustment
                    targetDistance = (((640.0/(currentWidth*getBoxWidth()))*1.27)/(0.260284))-Math.pow(0.93,currentWidth-50) - 2 ; //This is the full distancefrom the pole in CENTImeters!

                    //TODO: After curve fitting, this is some simple double-read code
                    if(currentWidth < 25){
                        driveForDistance((targetDistance/100) - 0.25, FORWARD, 0.25, imuHeading);
                        level2Aligned = false;
                    }else if(currentWidth > 40){
                        driveForDistance(0.11, BACKWARD, 0.2, imuHeading);
                        level2Aligned = false;
                    }else{
                        driveForDistance((targetDistance- 1.5 - 15)/100, FORWARD, 0.2, imuHeading);
                        level2Aligned = true;
                        levelCounter = 3;
                    }



                    telemetry.addLine("Target Distance: " + targetDistance + " cm");
                    telemetry.addLine("Boxes Width: " + currentWidth);

//                    driveForDistance(targetDistance/100.0, Direction.FORWARD, 0.2, imuHeading); //Get on top of the pole while using the imu
//
//
//                    levelCounter = 3;
//                    level2Aligned = true;
//                    telemetry.addLine("Level2 Complete!");



                }

//                if(level2Aligned){
//                    levelCounter = 3;
//                }


//                if(levelCounter == 2 && level2Aligned == false){ //TODO feed different inputs into this to make more aggressive
//
//
//                     motorFL.setPower(alignPowerAddedWidth - alignPowerAddedX/2.75);
//                     motorBL.setPower(alignPowerAddedWidth - alignPowerAddedX/2.75);
//                     motorFR.setPower(alignPowerAddedWidth + alignPowerAddedX/2.75);
//                     motorBR.setPower(alignPowerAddedWidth + alignPowerAddedX/2.75);
//                }

                    if (levelCounter == 3 && level3Assignment && getPercentColor() < 10) {
                        level3Aligned = true;
                        telemetry.addLine("We're at the top of the pole!");
                        telemetry.addLine("level3Aligned: " + level3Aligned);
                        telemetry.addLine("Percent Color: " + getPercentColor());
                        telemetry.update();
//                    sleep(1000);


                    }

                    if (levelCounter == 3 && level3Aligned == false) {
                        clawClamp();
                        motorSlide.setPower(0.65);
                        slidePosition = motorSlide.getCurrentPosition();
                        telemetry.addLine("Measuring the pole height!");
                        telemetry.addLine("Slide Position: " + motorSlide.getCurrentPosition());
                        telemetry.addLine("Percent Color: " + getPercentColor());

                        //Slide go up <-- Honestly just use a consistent power for ease
                    }

                    //For all the marbles, this is the sequence that stacks
                    if (level3Aligned == true) {
                        slidePosition = motorSlide.getCurrentPosition();
                        stopMotors();
                        telemetry.addLine("We going to the top babeeeeeeee");
                        telemetry.addLine("Slide position: " + slidePosition);
                        telemetry.addLine("targetHeight: " + targetHeight);
                        telemetry.update();
//                    sleep(500);
                        if (slidePosition >= 0 && slidePosition <= 1300) {
                            targetHeight = lowHeight;
                        } else if (slidePosition > 1300 && slidePosition <= 2500) {
                            targetHeight = midHeight;
                        } else if (slidePosition > 2500) {
                            targetHeight = highHeight;
                        }

                        clawClamp();
                        GoToHeight(targetHeight);
                        sleep(300);
                        driveForDistance(0.15, FORWARD, 0.2, imuHeading);
//                    sleep(250);
                        GoToHeight(targetHeight - 75);
                        sleep(350);
                        clawRelease();
                        sleep(200);
                        GoToHeight(targetHeight);
                        sleep(300);
                        driveForDistance(0.15, BACKWARD, 0.2, imuHeading);
                        goToDown();

                        levelCounter = 1;
                        level1Aligned = false;
                        level2Aligned = false;
                        level3Aligned = false;
                        visionAutoActivated = false;
                        targetX = 225; //TODO Avoid hard coding this value? Or maybe just take from the original resolution setting above
//                    dividerX = 200;
                        //Back to manual driving!!!

                    }
                }


//            if(level.equals("two") && Math.abs(errorX) < 8){
//                level2Aligned = true;
//            }


//            if(level2Aligned == false){
////                motorFL.setPower(alignPowerAdded);
////            motorBL.setPower(alignPowerAdded);
////            motorFR.setPower(-alignPowerAdded);
////            motorBR.setPower(-alignPowerAdded);
//
//            }


                if (levelCounter == 1) {
                    level = "one";
                }

                if (levelCounter == 2) {
                    level = "two";
                }

                if (levelCounter == 3) {
                    level = "three";
                }

            telemetry.addLine("CAN ACTIVATE?: " + getLevel2Capable());
            telemetry.addLine("Has been activated?: " + visionAutoActivated);
            telemetry.addLine("Current Level: " + getLevelString());
            telemetry.addLine("Current Width (boxes): " + currentWidth);
                telemetry.addLine("Box Width: " + getBoxWidth());
                telemetry.addLine("Current Width (pixels): " + currentWidth*getBoxWidth());
//            telemetry.addLine("Level2 Assignment? : " + getLevel2Assigment());
                telemetry.addLine("Percent Color: " + getPercentColor());
//                telemetry.addLine("LowestX: " + getLowestX());
//                telemetry.addLine("LowestY: " + getLowestY());
//                telemetry.addLine("HighestX: " + getHighestX());
//                telemetry.addLine("HighestY: " + getHighestY());
                telemetry.addLine("targetX: " + targetX);
                telemetry.addLine("centerX: " + getCenterX());
                telemetry.addLine("Power Applied X: " + alignPowerAddedX);
                telemetry.addLine("Power Applied Width: " + alignPowerAddedWidth);
//            telemetry.addLine("level1Aligned?: " + level1Aligned);
//                telemetry.addLine("level3Aligned?: " + level3Aligned);
//            telemetry.addLine("Slide Position: " + slidePosition);
//            telemetry.addLine("Largest Size: " + getLargestSize());


                telemetry.update();

                pipePoleTracker = new PipePoleTracker(level);
                camera.setPipeline(pipePoleTracker);
            }
        }



    public void rotateSmallWithBrake(double heading){
        double target = heading;
        double current = getCumulativeZ();
        double error = target - current;
        int sign = 0;

        int brakeWindow = (int)(error/55)*19;
        double maxPower = Math.abs(error/55);
        int reversalTime = (int)(error/55)*55; // This is in milliseconds (for sleep command)

        if((Math.abs(error/55)) > 1){
            brakeWindow = 19;
            maxPower = 1;
            reversalTime = 55;
        }else{
            maxPower = maxPower*0.85;
        }

        if(maxPower < 0.14){
            maxPower = 0.14;
        }

//        if(Math.abs(error) < 50 && Math.abs(error) >= 30){
//            maxPower= maxPower*0.85;
//        }

        while(Math.abs(error) > 1){
            current = getCumulativeZ();
            error = target - current;
            sign = (int)(error/Math.abs(error));

            motorFL.setPower(sign*-maxPower);
            motorBL.setPower(sign*-maxPower);
            motorFR.setPower(sign*maxPower);
            motorBR.setPower(sign*maxPower);

            telemetry.addLine("Current: " + getCumulativeZ());
            telemetry.addLine("Target: " + target);
            telemetry.addLine("Error: " + error);
            telemetry.addLine("Max Power: " + maxPower);
            telemetry.addLine("Brake Window: " + brakeWindow);
            telemetry.addLine("Reversal Time: " + reversalTime);
            telemetry.update();


            if(Math.abs(error) < brakeWindow){
                sign = (int)(error/Math.abs(error));

                motorFL.setPower(sign*maxPower);
                motorBL.setPower(sign*maxPower);
                motorFR.setPower(sign*-maxPower);
                motorBR.setPower(sign*-maxPower);
                sleep(reversalTime);
                break;
            }
        }
        stopMotors();

    }
}
