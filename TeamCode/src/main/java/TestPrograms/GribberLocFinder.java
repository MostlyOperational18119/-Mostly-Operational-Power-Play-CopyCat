package TestPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Timer;

//Created by Gabriel to Find Location of GG. Please run and read telemetry for usage instructions
@TeleOp(name="gribberLocFinder", group = "a")
@Disabled
public class GribberLocFinder extends LinearOpMode {

    Servo gribber;
    public void runOpMode() {
        gribber = hardwareMap.get(Servo.class, "grabber");
        double clampPos = 0.0;
        double releasePos = 0.0;
        double currentLoc = 0.0;
        int ajustTimes = 1;
        boolean mode1 = true;
        boolean release = true;
        Timer wait = new Timer();
        waitForStart();
        wait.startTimer();
        while(opModeIsActive()) {
            if (mode1) {
                if (wait.getTime() >= 3){
                    currentLoc += 0.05;
                    if (currentLoc > 1) {
                        currentLoc = 0.0;
                    }
                    currentLoc = ((int) (currentLoc * 100));
                    currentLoc = ((double) currentLoc) / 100;
                    gribber.setPosition(currentLoc);
                    wait.resetTimer();
                }
                if (gamepad1.a) {
                    mode1 = false;
                }
                if (mode1 && gamepad1.right_bumper){
                    clampPos = currentLoc;
                }
                if (mode1 && gamepad1.left_bumper) {
                    releasePos = currentLoc;
                }
            }
            else {
                if (release && wait.getTime() >= 3) {
                    gribber.setPosition(clampPos);
                    release = false;
                    currentLoc = clampPos;
                    wait.resetTimer();
                }
                if (!release && wait.getTime() >= 3) {
                    gribber.setPosition(releasePos);
                    release = true;
                    currentLoc = releasePos;
                    wait.resetTimer();
                }
                if (gamepad1.b) {
                    mode1 = true;
                    currentLoc = 0.00;
                }
                if (gamepad1.x) {ajustTimes = 5;}
                else {ajustTimes = 1;}
                if (gamepad1.dpad_up) {
                    releasePos += 0.01 * ajustTimes;
                    sleep(300);
                }
                if (gamepad1.dpad_down) {
                    releasePos -= 0.01 * ajustTimes;
                    sleep(300);
                }
                if (gamepad1.dpad_right) {
                    clampPos += 0.01 * ajustTimes;
                    sleep(300);
                }
                if (gamepad1.dpad_left) {
                    clampPos -= 0.01 * ajustTimes;
                    sleep(300);
                }
                if (clampPos > 1) {
                    clampPos = 1;
                }
                if (clampPos < 0) {
                    clampPos = 0;
                }
                if (releasePos > 1) {
                    releasePos = 1;
                }
                if (releasePos < 0) {
                    releasePos = 0;
                }
                clampPos = Math.round(clampPos * 100);
                clampPos = ((double) clampPos) / 100;
                releasePos = Math.round(releasePos * 100);
                releasePos = ((double) releasePos) / 100;
            }


            //final telemetry data
            telemetry.addData("Saved Release Pos", releasePos);
            telemetry.addData("Saved Clamp Pos", clampPos);
            if (!mode1 && release) {telemetry.addData("Released at", currentLoc);}
            if (!mode1 && !release) {telemetry.addData("Clamped at", currentLoc);}
            if (!mode1 && ajustTimes == 5) {telemetry.addLine("*Currently Ajusting by 0.05*");}
            if (mode1) {telemetry.addData("Current Location", currentLoc);telemetry.addLine("\nCurrently in Mode 1\nPress A to switch mode\nPress Left Bumper to record release position\nPress Right Bumper to record clamp position");}
            if (!mode1) {telemetry.addLine("\nCurrently in Mode 2\nPress B to switch mode\nUse Up & Down Pad to adjust release position\nUse Left & Right Pad to adjust clamp position\nHold X to ajust by 0.05");}
            telemetry.update();
        }
    }
}
