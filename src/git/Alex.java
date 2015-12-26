/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package git;




public class Alex {
    
    //ablage für code der gerade nicht verwendet wird
    //macht klassen übersichtlicher
    
    /*
    
    public void move(){
        double height = this.getBattleFieldHeight();
        double wide = this.getBattleFieldWidth();
//        double yBuffer = DANGER_ZONE * height;
        double yBuffer = 120;
        double xBuffer = 120;
        double turnAngle = 45;

        double xPosition = this.getX();
        double yPosition = this.getY();
        double direction = this.getHeading();
        System.out.println("Ich fahre");
//        out.println(" die Position X=" + xPosition + " Y=" + yPosition);

        if ((yPosition < yBuffer)) {

            System.out.println("Gefahr");

            if ((this.getHeading() < 180) && (this.getHeading() > 90)) {

                this.setTurnLeft(turnAngle);
            } else if ((this.getHeading() < 270) && (this.getHeading() > 180)) {

                this.setTurnRight(turnAngle);

            }

        } else if (yPosition > height - yBuffer) {
            System.out.println("Gefahr");
            if ((this.getHeading() < 90) && (this.getHeading() > 0)) {

                this.setTurnRight(turnAngle);
            } else if ((this.getHeading() < 360) && (this.getHeading() > 270)) {

                this.setTurnLeft(turnAngle);
            }
        } else if (xPosition < xBuffer) {
            System.out.println("Gefahr");
            if ((this.getHeading() > 180) && (this.getHeading() < 270)) {

                this.setTurnLeft(turnAngle);
            } else if ((this.getHeading() > 270) && (this.getHeading() < 360)) {

                this.setTurnRight(turnAngle);
            }
        } else if (xPosition > wide - xBuffer) {
            System.out.println("Gefahr");
            if ((this.getHeading() > 0) && (this.getHeading() < 90)) {

                this.setTurnLeft(turnAngle);
            } else if ((this.getHeading() > 90) && (this.getHeading() < 180)) {

                this.setTurnRight(turnAngle);
            }
        } else {
            this.setTurnRight(0);
            this.setTurnLeft(0);
        }

        setAhead(50);
        execute();
    }
    
    public void move1(){
        /Anti-Gravity Movement
        //zugriff auf hashtable: http://www.java2novice.com/java-collections-and-util/hashtable/iterate/
        //fast 1 zu 1 von: http://robowiki.net/wiki/Anti-Gravity_Tutorial
        //Wände noch nicht eingebaut
    
        xForce = 0;
        yForce = 0;
        Set keys = robots.keySet();
        for (Object key : keys) {
            RobotInfo robot = (RobotInfo) robots.get(key);
            if (!robot.ISTEAMMATE) {
                double absBearing = Utils.normalAbsoluteAngle(Math.atan2(robot.getX() - this.getX(), robot.getY() - this.getY()));
                double distance = robot.distance(getX(), getY());
                xForce -= Math.sin(absBearing) / (distance * distance);
                yForce -= Math.cos(absBearing) / (distance * distance);
                
            }
        }
        xForce -= 1/Math.pow(getRange(getX(), getY(), getBattleFieldWidth(), getY()), 2);
        //System.out.println("xForce - : " + 5000/Math.pow(getRange(getX(), getY(), getBattleFieldWidth(), getY()), 3));
        xForce += 1/Math.pow(getRange(getX(), getY(), 0, getY()), 2);
        //System.out.println("xForce + : " + 5000/Math.pow(getRange(getX(), getY(), 0, getY()), 3));
        yForce -= 1/Math.pow(getRange(getX(), getY(), getX(), getBattleFieldHeight()), 2);
        //System.out.println("yForce - : " + 5000/Math.pow(getRange(getX(), getY(), getX(), getBattleFieldHeight()), 3));
        yForce += 1/Math.pow(getRange(getX(), getY(), getX(), 0), 2);
        //System.out.println("yForce + : " + 5000/Math.pow(getRange(getX(), getY(), getX(), 0), 3));
        //System.out.println("xForce: " + xForce + "; yForce: " + yForce);
        angle = Math.atan2(xForce, yForce);
        double pyth = Math.sqrt(xForce*xForce+ yForce*yForce);
                System.out.println("xForce = " + 10000* xForce);
                System.out.println("                   -     yForce = " +10000*yForce);
        destination = new Point2D.Double(getX() + 120*xForce/pyth, getY() + 120*yForce/pyth);
        double speed = 100;
        if (getX()<100 || this.getBattleFieldWidth()-getX()<100 || getY()<100 || this.getBattleFieldHeight()-getY()<100){
            speed /= 10;
        }
        
        
        if (xForce == 0 && yForce == 0) {
            setAhead(1.0); //kleiner Wert damit wir nicht stehen
        } else if (Math.abs(angle - getHeadingRadians()) < Math.PI / 2) {
            setTurnRightRadians(Utils.normalRelativeAngle(angle - getHeadingRadians()));
            setAhead(speed);
            //setAhead(Double.POSITIVE_INFINITY);
            //System.out.println("Vorwärts");
        } else {
            setTurnRightRadians(Utils.normalRelativeAngle(angle + Math.PI - getHeadingRadians()));
            setAhead(-speed);
            //setAhead(Double.NEGATIVE_INFINITY);
            //System.out.println("Rückwärts");
        }
        
    
    }
    
    */
    
}
