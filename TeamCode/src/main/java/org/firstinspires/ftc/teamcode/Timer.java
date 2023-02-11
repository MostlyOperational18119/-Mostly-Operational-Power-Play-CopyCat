package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Timer {
    ElapsedTime daTime = new ElapsedTime();
    Boolean timerStopped = true;
    double cTime = 6.9;
    public void startTimer() {
        daTime.reset();
        timerStopped = false;
    }
    public void resetTimer() {
        daTime.reset();
    }
    public void stopTimer() {
        cTime = daTime.seconds();
        timerStopped = true;
    }
    public double getTime() {
        if (!timerStopped) {
            cTime = daTime.seconds();
        }
        return cTime;
    }
    public void cleanup() {
        stopTimer();
        timerStopped =  null;
        daTime = null;
    }
}
