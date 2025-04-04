// Helper function to convert VSG matrix rotation to ODE format
void odeRotation(const vsg::dmat4& mat, dMatrix3& odematrix) {
    // ODE matrix is stored in row-major order with indices:
    // [0 1 2 3]    0  1  2  (3 unused)
    // [4 5 6 7] =  4  5  6  (7 unused)
    // [8 9 10 11]  8  9  10 (11 unused)
    
    // Copy rotation components (transposing from VSG column-major to ODE row-major)
    odematrix[0] = mat[0][0]; odematrix[1] = mat[1][0]; odematrix[2] = mat[2][0];
    odematrix[4] = mat[0][1]; odematrix[5] = mat[1][1]; odematrix[6] = mat[2][1];
    odematrix[8] = mat[0][2]; odematrix[9] = mat[1][2]; odematrix[10] = mat[2][2];
    
    // Set unused elements to 0
    odematrix[3] = odematrix[7] = odematrix[11] = 0.0;
}

// Overload for Pose input
void odeRotation(const Pose& pose, dMatrix3& odematrix) {
    odeRotation(pose.getMatrix(), odematrix);
}

// Complete transform setter using the rotation conversion
void Primitive::setmat(const vsg::dmat4& mat) {
    if (body) {
        // Set position using the translation component (last column in VSG matrix)
        dBodySetPosition(body, mat[3][0], mat[3][1], mat[3][2]);
        
        // Set rotation
        dMatrix3 R;
        odeRotation(mat, R);
        dBodySetRotation(body, R);
    } 
    else if (geom) {
        // Set position
        dGeomSetPosition(geom, mat[3][0], mat[3][1], mat[3][2]);
        
        // Set rotation
        dMatrix3 R;
        odeRotation(mat, R);
        dGeomSetRotation(geom, R);
    }
    
    // Update VSG transform
    if (transformNode) {
        transformNode->matrix = mat;
    }
    
    update();
}

// attention: This sox here is now called SoML in selforg/controller
    SoXConf sc = SoX::getDefaultConf();
    sc.useHiddenContr=true;
    sc.useHiddenModel=true;
    sc.someInternalParams=true;
    sc.useS=true;
    AbstractController* controller = new SoX(sc);
    controller->setParam("epsC",0.05);
    controller->setParam("epsA",0.05555555);
    controller->setParam("harmony",0.0);
    controller->setParam("s4avg",5.0);

// OTHER CONTROLLERS TO EXPORT NEXT STEP
    //     AbstractController* controller = new BasicController(cc);
    //   AbstractController* controller = new SineController(1<<14); // only motor 14
    // controller = new MotorBabbler();
    //AbstractController* controller = new SineController(0,SineController::Impulse);

    global.configs.push_back(controller);

    // create pointer to one2onewiring
    One2OneWiring* wiring = new One2OneWiring(new WhiteUniformNoise());
    // DerivativeWiringConf c = DerivativeWiring::getDefaultConf();
    // c.useId = true;
    // c.useFirstD = false;
    // DerivativeWiring* wiring = new DerivativeWiring ( c , new ColorUniformNoise(.2) );

    OdeAgent* agent = new OdeAgent(global);
    agent->init(controller, human, wiring);
    //agent->startMotorBabblingMode(5000);
    //agent->setTrackOptions(TrackRobot(true,true,false,true,"bodyheight",20)); // position and speed tracking every 20 steps
    global.agents.push_back(agent);