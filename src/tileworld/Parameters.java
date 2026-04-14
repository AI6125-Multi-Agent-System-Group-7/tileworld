package tileworld;

/**
 * Parameters
 *
 * @author michaellees
 * Created: Apr 21, 2010
 *
 * Copyright michaellees 
 *
 * Description:
 *
 * Class used to store global simulation parameters.
 * Environment related parameters are still in the TWEnvironment class.
 *
 */
public class Parameters {
    // Ablation switch: when true, environment drops all inter-agent messages.
    // Default false to keep normal communication behavior.
    public static final boolean messageBlackholeEnabled = false;

    //Simulation Parameters 1 ===================================================
    // public final static int seed = 4162012; //no effect with gui
    // public static final long endTime = 5000; //no effect with gui

    // //Agent Parameters
    // public static final int defaultFuelLevel = 500;
    // public static final int defaultSensorRange = 3;

    // //Environment Parameters
    // public static final int xDimension = 50; //size in cells
    // public static final int yDimension = 50;

    // //Object Parameters
    // // mean, dev: control the number of objects to be created in every time step (i.e. average object creation rate)
    // public static final double tileMean = 0.2;
    // public static final double holeMean = 0.2;
    // public static final double obstacleMean = 0.2;
    // public static final double tileDev = 0.05f;
    // public static final double holeDev = 0.05f;
    // public static final double obstacleDev = 0.05f;
    // // the life time of each object
    // public static final int lifeTime = 100;


    //Simulation Parameters 2 ===================================================
    // public final static int seed = 4162012; //no effect with gui
    // public static final long endTime = 5000; //no effect with gui

    // //Agent Parameters
    // public static final int defaultFuelLevel = 500;
    // public static final int defaultSensorRange = 3;

    // //Environment Parameters
    // public static final int xDimension = 80; //size in cells
    // public static final int yDimension = 80;

    // //Object Parameters
    // // mean, dev: control the number of objects to be created in every time step (i.e. average object creation rate)
    // public static final double tileMean = 2;
    // public static final double holeMean = 2;
    // public static final double obstacleMean = 2;
    // public static final double tileDev = 0.5f;
    // public static final double holeDev = 0.5f;
    // public static final double obstacleDev = 0.5f;
    // // the life time of each object
    // public static final int lifeTime = 30;
    
    //Simulation Parameters 3 ===================================================
    public final static int seed = 4162012; //no effect with gui
    public static final long endTime = 5000; //no effect with gui

    //Agent Parameters
    public static final int defaultFuelLevel = 500;
    public static final int defaultSensorRange = 3;

    //Environment Parameters
    public static final int xDimension = 120; //size in cells
    public static final int yDimension = 120;

    //Object Parameters
    // mean, dev: control the number of objects to be created in every time step (i.e. average object creation rate)
    public static final double tileMean = 5;
    public static final double holeMean = 5;
    public static final double obstacleMean = 5;
    public static final double tileDev = 0.5f;
    public static final double holeDev = 0.5f;
    public static final double obstacleDev = 0.5f;
    // the life time of each object
    public static final int lifeTime = 50;


    //Simulation Parameters myth ===================================================
    // public final static int seed = 4162010; //no effect with gui
    // public static final long endTime = 5000; //no effect with gui

    // //Agent Parameters
    // public static final int defaultFuelLevel = 500;
    // public static final int defaultSensorRange = 3;

    // //Environment Parameters
    // public static final int xDimension = 100; //size in cells
    // public static final int yDimension = 40;

    // //Object Parameters
    // // mean, dev: control the number of objects to be created in every time step (i.e. average object creation rate)
    // public static final double tileMean = 2;
    // public static final double holeMean = 2;
    // public static final double obstacleMean = 2;
    // public static final double tileDev = 0.5f;
    // public static final double holeDev = 0.5f;
    // public static final double obstacleDev = 0.5f;
    // // the life time of each object
    // public static final int lifeTime = 36;

}
