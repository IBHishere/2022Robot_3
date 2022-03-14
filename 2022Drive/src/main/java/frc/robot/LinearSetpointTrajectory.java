package frc.robot;

import java.util.Date;

public class LinearSetpointTrajectory {
    private double m_startPosition;
    private double m_endPosition;
    private double m_timeInMillisecondsToComplete;
    private Date m_startTime;
    private Date m_endTime;
    private boolean isStarted = false;
    private double forwardTimeStep = 100;

    public LinearSetpointTrajectory(
        double startPosition,
        double endPosition,
        double timeInMillisecondsToComplete
    ) {
        System.out.println("constructor, " + new Date().getTime());
        this.m_startPosition = startPosition;
        this.m_endPosition = endPosition;
        this.m_timeInMillisecondsToComplete = timeInMillisecondsToComplete;
    }

    private void initTiming() {
        this.m_startTime = new Date();
        this.m_endTime= new Date(this.m_startTime.getTime()+ (long) this.m_timeInMillisecondsToComplete);
    }

    public boolean isSetpointSettingDone() {
        Date currentDate = new Date();
        return currentDate.after(this.m_endTime);
    }

    public double getSetpoint() {
        if(!this.isStarted) {
            initTiming();
            this.isStarted = true;
        }

        Date currentDate = new Date();
        return calculateDesiredPositionAtTime(currentDate.getTime() + this.forwardTimeStep);
    }

    private double calculateDesiredPositionAtTime(double milliseconds) {
        // compute disired position as a function of time - instead of changing the setpoint quickly
        // we want to transition gradually over time.  So for some period of time (specified in the contructor)
        // the setpoint is changed from staring to end pooint linearly. 
        
        double desiredPosition = this.m_endPosition; //default to end setpoint 

        System.out.print("setpoint, "+this.m_startTime.getTime() + ", " + milliseconds+","+this.m_endTime.getTime());

        //if during transition time then compute the position
        if(milliseconds <  this.m_endTime.getTime()) { 
            double msSinceStart = milliseconds - this.m_startTime.getTime();
            double percentComplete = msSinceStart/this.m_timeInMillisecondsToComplete;
            System.out.print(", " + msSinceStart + ", " + percentComplete);
            desiredPosition = this.m_startPosition +  
                       (this.m_endPosition-this.m_startPosition) * percentComplete;
        }
        
        System.out.println(", "+desiredPosition);

        return desiredPosition;       
    }
}