```java
package ultimaterobot;

import robocode.*;
import robocode.util.*;
import java.awt.*;
import java.awt.geom.*;
import java.lang.Math;



public class Shredder extends AdvancedRobot{
	
	private int moveSpeed; //Speed of the robot
	private int moveDirection; // 1 move forward, -1 move back
	private int turnDirection; // 1 turn right, -1 turn left
	private Point2D location; // Current location of the robot
	private Point2D securePointRight; // Current location of the right secure point 
	private Point2D securePointLeft; // Current location of the left secure point
	private int enemies; // Current number of enemies
	private int wallMargin; // Distance between the center of the robot and the secure points
	private Rectangle2D.Double battleField; // The battlefield
	private Condition triggerCloseWall; // The condition to detect the walls
	
	/**
	 * run: CamberoPiqueras's default behavior
	 */
	public void run() {
		
		initialize();
		
		// Robot main loop
		while(true) {
			if ( getRadarTurnRemaining() == 0.0 ){
            	setTurnRadarRightRadians( Double.POSITIVE_INFINITY );
 			}
			this.moveDirection = 1;
			setAhead(1000 * this.moveDirection);
			execute();		
		}
	}
	
	/**
	 * initialize: Initialize the robot atributes
	 */
	private void initialize(){
		this.moveSpeed = 5;
		this.moveDirection = 1;
		this.turnDirection = 1;
		this.location = new Point2D.Double(getX(), getY());
		this.securePointRight = new Point2D.Double(getX(), getY());
		this.securePointLeft = new Point2D.Double(getX(), getY());
		this.enemies = getOthers();
		this.wallMargin = 150;
		this.battleField = new Rectangle2D.Double(0, 0, getBattleFieldWidth(), getBattleFieldHeight());
		
		triggerCloseWall = new Condition("triggerclosewall") {
      		 public boolean test() {
		        return closeWall();
		     }
	   };
		
		setColors(Color.red, Color.yellow, Color.red, Color.green, Color.green); // body, gun, radar, bullet, scannAcr
	    triggerCloseWall.setPriority(35);
	    addCustomEvent(triggerCloseWall);
		setAdjustGunForRobotTurn(true);
		setAdjustRadarForGunTurn(true);
		setMaxVelocity(this.moveSpeed);
	}
	
	/**
	 * updateLocation: Update the position of the robot and its secure points
	 */
	private void updateLocation(){
		this.location.setLocation(getX(), getY());
		double bodyHeading = getHeadingRadians();
		double distance = (this.wallMargin/Math.sqrt(2));
		double xs1 = this.location.getX() + ((distance*Math.cos(bodyHeading)) + (distance*Math.sin(bodyHeading)));
		double ys1 = this.location.getY() + ((distance*Math.cos(bodyHeading)) - (distance*Math.sin(bodyHeading)));
		double xs2 = this.location.getX() + ((-distance*Math.cos(bodyHeading)) + (distance*Math.sin(bodyHeading)));
		double ys2 = this.location.getY() + ((distance*Math.cos(bodyHeading)) - (-distance*Math.sin(bodyHeading)));
		this.securePointRight.setLocation(xs1, ys1);
		this.securePointLeft.setLocation(xs2, ys2);
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
	    double angleToEnemy = getHeadingRadians() + e.getBearingRadians();
		
	    double radarTurnAngle = Utils.normalRelativeAngle(angleToEnemy - getRadarHeadingRadians());
	    // Distance we want to scan from middle of enemy to either side
	    // The 36.0 is how many units from the center of the enemy robot it scans.
	    double extraTurn = Math.min( Math.atan( 40.0 / e.getDistance() ), Rules.RADAR_TURN_RATE_RADIANS );
	
	    // Adjust the radar turn so it goes that much further in the direction it is going to turn
	    if (radarTurnAngle < 0){
	        radarTurnAngle -= extraTurn;
		}else{
	        radarTurnAngle += extraTurn;
		}
	 
	    //Turn the radar
	    setTurnRadarRightRadians(radarTurnAngle);
		
		double gunTurnAngle = Utils.normalRelativeAngle(angleToEnemy - getGunHeadingRadians());
		double enemyLateralSpeed = e.getVelocity() * Math.sin(e.getHeadingRadians() - angleToEnemy);
		// 30.0 is the factor to reduce the offset
		double offset = (enemyLateralSpeed / 40.0);
		
		gunTurnAngle += offset;
		setTurnGunRightRadians(gunTurnAngle);

		// If it's close enough, fire!
		if (Math.abs(gunTurnAngle) <= 3 && getGunHeat() == 0) {
				setFire(decideFirePower(e.getDistance(), e.getEnergy(), Math.abs(Math.toDegrees(gunTurnAngle))));
		}
		// Generates another scan event if we see a robot.
		if (gunTurnAngle == 0) {
			scan();
		}
		
		// Move around the detected robot
		if (e.getBearing() > 0) { 			
			setTurnRight(Utils.normalRelativeAngleDegrees(e.getBearing() - 90)); 
			this.turnDirection = 1;		
		} else { 			
			setTurnLeft(Utils.normalRelativeAngleDegrees(e.getBearing() + 90)); 		
			this.turnDirection = -1;
		}
		this.moveDirection = 1;
		setAhead(1000 * this.moveDirection);
		
		execute();
	}
	
	/**
	 * onHitRobot: What to do when you hit an other robot
	 */
	public void onHitRobot(HitRobotEvent e) {
		stop();
		double gunTurnAngle = getHeading() - getGunHeading() + e.getBearing();
		double radarTurnAngle = getHeading() - getRadarHeading() + e.getBearing();
       if (Math.abs(gunTurnAngle) < 90) {
	   		System.out.println("Cerca de disparo");
	   		// If the gun is near of the hit direction, turn the gun and shoot
           turnGunRight(gunTurnAngle);
		   if (Math.abs(getGunTurnRemaining()) < 10 && getGunHeat() == 0){
		   		System.out.println("Disparo");
		   		// To avoid waste one turn and energy, check if the gun is cold and if it`s in the right angle 
				setFire(decideFirePower(0, e.getEnergy(), Math.abs(gunTurnAngle)));
			}
       }
		if (e.isMyFault() == true) {
			// If the hit was ahead, go back
		   this.moveDirection = -1;		System.out.println("Hacia atras");   
       } else {
		   this.moveDirection = 1;System.out.println("Hacia delante");
	   }
	   setTurnRadarRight(radarTurnAngle);
       setAhead(100 * this.moveDirection);
	   execute();
   }
   
	/**
	 * decideFirePower: With what power to shot depending on the circumstances
	 */
	private double decideFirePower(double enemyDistance, double enemyEnergy, double gunTurnAngle){
		double enemyDistanceInPowerRange = map(enemyDistance, 500, 0, 0.1, 3);
		double enemyEnergyInPowerRange = map(enemyEnergy, 0.1, 3, 0.1, 3);
		double gunTurnAngleInPowerRange = map(gunTurnAngle, 180, 20, 0.1, 3);
		double robotEnergyInPowerRange = map(this.getEnergy(), 0.5, 20, 0.1, 3);
		double firePower = ((enemyDistanceInPowerRange * 0.30) + (enemyEnergyInPowerRange * 0.025) + (gunTurnAngleInPowerRange * 0.35) + (robotEnergyInPowerRange * 0.025));
		System.out.println("FirePower: " + firePower);
		return firePower;
	}
	
	/**
	 * map: Converts one value into a input range into another within an output range
	 */
	private double map(double inputValue, double startInputRange, double endInputRange, double startOutputRange, double endOutputRange){
		if(startInputRange < endInputRange){
			if(inputValue < startInputRange){
				inputValue = startInputRange;
			}else if(inputValue > endInputRange){
				inputValue = endInputRange;
			}
		} else {
			if(inputValue > startInputRange){
				inputValue = startInputRange;
			}else if(inputValue < endInputRange){
				inputValue = endInputRange;
			}
		}
		double outputValue = (((inputValue - startInputRange) * ((endOutputRange - startOutputRange) / (endInputRange - startInputRange)))+ startOutputRange);
		return outputValue;
	}
   
	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		// Try to avoid more bullets in the same direction
		this.turnDirection = 1;
		setTurnRight(45 * this.turnDirection);
	}
	
	/**
	 * onHitWall: When the robot hit a wall, the robot turns around
	 */
	public void onHitWall(HitWallEvent e) {
		stop();
		this.turnDirection = 1;
		setTurnRight(180 * this.turnDirection);
		waitFor(new TurnCompleteCondition(this));
	}	
	
	/**
	 * onCustomEvent: What to do on custom events
	 */
	public void onCustomEvent(CustomEvent e) {
		if (e.getCondition().getName().equals("triggerclosewall")) {
			// What to do when you are near a wall
			this.updateLocation();
			if((!this.battleField.contains(this.securePointRight)) && (!this.battleField.contains(this.securePointLeft))){
				// If the two points are outside of the battlefield, the robot is near of a corner or perpendicular to the wall
				this.turnDirection = 1;
				setTurnRight(180 * this.turnDirection);
			}else if(!this.battleField.contains(this.securePointRight)){
				// If the right secure point is outside of the battlefield, turn to left
				this.turnDirection = -1;
				setTurnRight(90 * this.turnDirection);
				this.moveDirection = 1;
				setAhead(10 * this.moveDirection);
			}else if(!this.battleField.contains(this.securePointLeft)){
				// If the left secure point is outside of the battlefield, turn to right
				this.turnDirection = 1;
				setTurnRight(90 * this.turnDirection);
				this.moveDirection = 1;
				setAhead(10 * this.moveDirection);
			}
			execute();
       	}
	}
	
	/**
	 * closeWall: Detects if the robot could hit a wall
	 */
	private boolean closeWall(){
		this.updateLocation();
		if((!this.battleField.contains(this.securePointRight)) || (!this.battleField.contains(this.securePointLeft))){
			return true;
		}else{
			return false;
		}
	}	
	
	/**
	 * onRobotDeath: What to do when an other robot dies
	 */
	public void onRobotDeath(RobotDeathEvent e) {
		this.enemies = getOthers();
		if (this.enemies > 10) {
			this.moveSpeed = 15;
			setMaxVelocity(this.moveSpeed);
		} else if (this.enemies > 1) {
			this.moveSpeed = 10;
			setMaxVelocity(this.moveSpeed);
		} else if (this.enemies == 1) {
			this.moveSpeed = 10;
			setMaxVelocity(this.moveSpeed);
		}
	}   
	
	/**
	 * onWin: The robot celebration
	 */
	public void onWin(WinEvent e) {
		stop();
		setTurnRadarLeft(0);
		setTurnRight(36000);
		execute();
	}
}

```
