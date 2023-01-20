package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.motorBL;
import static org.firstinspires.ftc.teamcode.Variables.motorBR;
import static org.firstinspires.ftc.teamcode.Variables.motorFL;
import static org.firstinspires.ftc.teamcode.Variables.motorFR;
import static org.firstinspires.ftc.teamcode.Variables.motorSlide;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Meet3Teleop", group = "A")
public class Meet3Teleop extends DriveMethods {

   


    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        initMotorsBlue();
        motorSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        double leftY;
        double leftX;
        double rightX;
        double speedDiv = 2;
        // Can we deleat Clamp & Relase Pos?
        // LOOK IN VARIABLES FOR GRIBBER POSISITIONS, SEE NUMBER ON GRIBBER
        //double clampPosition = 0.76;
        //double releasePosition = 0.66;
        double aggressiveness = 3000;
        double holdingPower = 0.05;
        int slideTarget = 0;
        int slideDifference = 0;
        int targetHeight = 0;
        double sPosition = motorSlide.getCurrentPosition();
        boolean isManualControl = true;
        int coneStackHeight = 6;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            sPosition = motorSlide.getCurrentPosition();

            //update doubles
            leftY = -gamepad1.left_stick_y;
            leftX = gamepad1.left_stick_x;
            rightX = gamepad1.right_stick_x;


            if (gamepad2.x) {
                clawClamp();
            }
            if (gamepad2.a) {
                clawRelease();
            }

            if (!gamepad1.right_bumper) {
                motorFL.setPower((leftY + leftX + rightX) / speedDiv);
                motorBL.setPower((leftY - leftX + rightX) / speedDiv);
                motorFR.setPower((leftY - leftX - rightX) / speedDiv);
                motorBR.setPower((leftY + leftX - rightX) / speedDiv);
            } else {
                motorFL.setPower(0);
                motorBL.setPower(0);
                motorFR.setPower(0);
                motorBR.setPower(0);
            }
            if (gamepad1.a) {
                speedDiv = 2;
            }
            if(gamepad1.b) {
                speedDiv = 4;
            }
            if(gamepad1.x) {
                speedDiv = 1.5;
            }
            if(gamepad2.left_trigger==1) {
                isManualControl = false;
            }
            if(gamepad2.right_trigger==1) {
                isManualControl = true;
            }
            if((gamepad2.dpad_up || gamepad2.dpad_down) & isManualControl){
                if(gamepad2.dpad_up) {
                    targetHeight++;
                    if (targetHeight > 4) {
                        targetHeight = 4;
                    }
                    sleep(150);
                }
                if (gamepad2.dpad_down) {
                    targetHeight--;
                    if (targetHeight < 0) {
                        targetHeight = 0;
                    }
                    sleep(150);
                }
                switch (targetHeight) {
                        case 0:
                            slideTarget = 0;
                            aggressiveness = 2000;
                            holdingPower = 0.0;
                            break;
                        case 1:
                            slideTarget = 200;
                            aggressiveness = 1000;
                            holdingPower = 0.06;
                            break;
                        case 2:
                            slideTarget = 1800;
                            aggressiveness = 2000;
                            holdingPower = 0.18;
                            break;
                        case 3:
                            slideTarget = 2950;
                            aggressiveness = 2000;
                            holdingPower = 0.18;
                            break;
                        case 4:
                            slideTarget = 4200;
                            aggressiveness = 2000;
                            holdingPower = 0.18;
                            break;

                    }
                }
            if((gamepad2.dpad_up || gamepad2.dpad_down) & !isManualControl) {
                if(gamepad2.dpad_up && coneStackHeight!=7) {
                    coneStackHeight++;
                    sleep(150);
                }
                if(gamepad2.dpad_down && coneStackHeight!=0) {
                    coneStackHeight--;
                    sleep(150);
                }
                //1283
                //615 for 5
                //460 for 4
                //290 for 3
                //190 for 2
                //000 for 1
                switch (coneStackHeight) {
                    case 0:
                    case 1:
                        slideTarget = 0;
                        aggressiveness = 2000;
                        holdingPower = 0;
                        break;
                    case 2:
                        slideTarget = 190;
                        aggressiveness = 2000;
                        holdingPower = 0.06;
                        break;
                    case 3:
                        slideTarget = 290;
                        aggressiveness = 1000;
                        holdingPower = 0.18;
                        break;
                    case 4:
                        slideTarget = 460;
                        aggressiveness = 2000;
                        holdingPower = 0.18;
                        break;
                    case 5:
                        slideTarget = 615;
                        aggressiveness = 2000;
                        holdingPower = 0.18;
                        break;
                    case 6:
                        slideTarget = 815;
                        aggressiveness = 2000;
                        holdingPower = 0.18;
                        break;
                    case 7:
                        slideTarget = 1300;
                        aggressiveness = 2000;
                        holdingPower = 0.18;
                        break;
                }
            }
            //Change the target height based on the height of the linear slide at the time.

            if(gamepad2.right_bumper){
                targetHeight = 4;
                sleep(50);
            }
            if(gamepad2.left_bumper){
                targetHeight = 0;
                sleep(50);
            }




            if(gamepad2.left_stick_y != 0){
                    slideTarget += (int) -gamepad2.left_stick_y * 40;
                    aggressiveness = 1250;
                    sleep(50);
                if (sPosition<300 && sPosition>0){
                    targetHeight = 1;
                }
                if (sPosition<1900 && sPosition>300){
                    targetHeight = 2;
                }
                if (sPosition<3150 && sPosition>1900){
                    targetHeight = 3;
                }
                if (sPosition>3150 && sPosition<4300){
                    targetHeight = 4;
                }
            }
            if(slideTarget<0){
                slideTarget=0;
            }
            if(slideTarget>4400){
                slideTarget = 4400;
            }

//            if(motorSlide.getCurrentPosition()<150){
//                aggressiveness = 1750;
//            }

            if(slideTarget == 0 && motorSlide.getCurrentPosition() < 150 && motorSlide.getCurrentPosition() >= 50){
                aggressiveness = 700;
                holdingPower = 0;
            }

            if(slideTarget == 0 && motorSlide.getCurrentPosition() < 50){
                aggressiveness = 400;
                holdingPower = 0;
            }


            slideDifference = (slideTarget - Math.abs(motorSlide.getCurrentPosition()));

            motorSlide.setPower(((slideDifference / aggressiveness) + holdingPower));

            telemetry.addLine(slideDifference + "..difference");
            telemetry.addLine(Math.abs(motorSlide.getCurrentPosition()) + "..position");
            telemetry.addLine(slideTarget + "..target");
            telemetry.addLine(((slideDifference / aggressiveness) + holdingPower) + "..power");
            telemetry.addLine("Target Height: " + targetHeight);

//            motorSlide.setPower(holdingPower);



            // Show the elapsed game time and wheel power.
            telemetry.addData("Motors", "left (%.2f), right (%.2f)");
            telemetry.addLine("SpeedDiv: " + speedDiv);
            telemetry.addLine("linear slide position " + motorSlide.getCurrentPosition());
            telemetry.addLine("left_stick_y_2: " + gamepad2.left_stick_y);
            telemetry.update();
        }
    }
}
