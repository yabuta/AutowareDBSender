
#ifndef AXIALMOVE
#define AXIALMOVE

#include <math.h>
#include "CalSelfLoc.h"

typedef struct _ANGLE{
    double thiX;
    double thiY;
    double thiZ;

}ANGLE;

typedef struct _MoveVector{
    double X;
    double Y;
    double Z;

}MoveVector;

class axiMove{

public:

    axiMove(){};

    LOCATION cal(LOCATION loc,ANGLE ang, MoveVector mv){

        LOCATION newloc;
        //rotation around X
        newloc.Y = loc.Y*cos(ang.thiX) - loc.Z*sin(ang.thiX);
        newloc.Z = loc.Y*sin(ang.thiX) + loc.Z*cos(ang.thiX);
        loc.Y = newloc.Y;
        loc.Z = newloc.Z;

        //rotation around Y
        newloc.Z = loc.Z*cos(ang.thiY) - loc.X*sin(ang.thiY);
        newloc.X = loc.Z*sin(ang.thiY) + loc.X*cos(ang.thiY);
        loc.X = newloc.X;
        loc.Z = newloc.Z;

        //rotation around Y
        newloc.X = loc.X*cos(ang.thiZ) - loc.Y*sin(ang.thiZ);
        newloc.Y = loc.Y*sin(ang.thiZ) + loc.Y*cos(ang.thiZ);
        loc.X = newloc.X;
        loc.Y = newloc.Y;

        loc.X = loc.X + mv.X;
        loc.Y = loc.Y + mv.Y;
        loc.Z = loc.Z + mv.Z;

        return loc;

    }

    LOCATION cal(LOCATION loc,ANGLE ang){

        LOCATION newloc;
        //rotation around X
        newloc.Y = loc.Y*cos(ang.thiX) - loc.Z*sin(ang.thiX);
        newloc.Z = loc.Y*sin(ang.thiX) + loc.Z*cos(ang.thiX);
        loc.Y = newloc.Y;
        loc.Z = newloc.Z;

        //rotation around Y
        newloc.Z = loc.Z*cos(ang.thiY) - loc.X*sin(ang.thiY);
        newloc.X = loc.Z*sin(ang.thiY) + loc.X*cos(ang.thiY);
        loc.X = newloc.X;
        loc.Z = newloc.Z;

        //rotation around Y
        newloc.X = loc.X*cos(ang.thiZ) - loc.Y*sin(ang.thiZ);
        newloc.Y = loc.Y*sin(ang.thiZ) + loc.Y*cos(ang.thiZ);
        loc.X = newloc.X;
        loc.Y = newloc.Y;

        return loc;

    }



};

#endif
