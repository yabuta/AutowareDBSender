
#ifndef CALCOOD
#define CALCOOD

#include <math.h>

typedef struct _RESULT{
    double lat;
    double log;

}RESULT;


//this calculation refer to http://vldb.gsi.go.jp/sokuchi/surveycalc/surveycalc/algorithm/xy2bl/xy2bl.htm
class calcoordinates{


public:
    calcoordinates(){};


    RESULT cal(double x0,double y0 ,double phizero, double lamzero){
        x = x0;
        y = y0;
        phi0 = phizero;
        lam0 = lamzero;
        
        //circle n depend on previous number circle.
         
        //circle 1
        double n = 1/(2*F-1); //*

        //circle 2  pow(n,)
        double B[6]; //*
        B[0] = 0;
        B[1] = n/2 - 2*n*n/3 + 37*pow(n,3)/96 - pow(n,4)/360 - 81*pow(n,5)/512;
        B[2] = n*n/48 + pow(n,3)/15 - 437*pow(n,4)/1440 + 46*pow(n,5)/105;
        B[3] = 17*pow(n,3)/480 - 37*pow(n,4)/840 - 209*pow(n,5)/4480;
        B[4] = 4397*pow(n,4)/161280 - 11*pow(n,5)/504;
        B[5] = 4583*pow(n,5)/161280;

        double A[6]; //*
        A[0] = 1 + n*n/4 + pow(n,4)/64;
        A[1] = -3*(n-pow(n,3)/8-pow(n,5)/64)/2;
        A[2] = 15*(n*n-pow(n,4)/4)/16;
        A[3] = -35*(pow(n,3)-5*pow(n,5)/16)/48;
        A[4] = 315*pow(n,4)/512;
        A[5] = -693*pow(n,5)/1280;
        
        double delta[7];
        delta[0] = 0;
        delta[1] = 2*n - 2*n*n/3 - 2*pow(n,3) + 116*pow(n,4)/45 + 26*pow(n,5)/45 * 2854*pow(n,6)/675;
        delta[2] = 7*n*n/3 - 8*pow(n,3)/5 - 227*pow(n,4)/45 + 2704*pow(n,5)/315 + 2323*pow(n,6)/945;
        delta[3] = 56*pow(n,3)/15 - 136*pow(n,4)/35 - 1262*pow(n,5)/105 + 73814*pow(n,6)/2835;
        delta[4] = 4279*pow(n,4)/630 - 332*pow(n,5)/35 - 399572*pow(n,6)/14175;
        delta[5] = 4174*pow(n,5)/315 - 144838*pow(n,6)/6237;
        delta[6] = 601676*pow(n,6)/22275;

        //circle 3
        double Abar = m0*a*A[0]/(1+n); //*

        double temp1 = A[0]*phi0/rowdd;
        for(int j=1; j<6 ; j++){
            temp1 += A[j]*sin(2*j*phi0);
        }
        double Spbar = m0*a*temp1/(1+n); //*

        //circle 4
        double xi = (x+Spbar)/Abar;
        double eta = y/Abar;
        
        //circle 5
        double xid = xi;
        for(int j=1 ; j<6 ; j++){
            xid -= B[j]*sin(2*j*xi)*cosh(2*j*eta); 
        }
        double etad = eta;
        for(int j=1 ; j<6 ; j++){
            etad -= B[j]*cos(2*j*xi)*sinh(2*j*eta); 
        }

        //circle 6
        double chi = asin(sin(xid)/cosh(etad));

        //result

        res.lat = chi;
        for(int j=1; j<7 ; j++){
            res.lat += delta[j]*sin(2*j*chi);
        }

        res.log = log0 + atan(sinh(etad)/cos(xid));
        
        return res;

    }

private:

    double x,y;
    double phi0,lam0;
    RESULT res;

    static const double a = 6378137;
    static const double F = 1.0/298.257222101;

    static const double m0 = 0.9999;
    static const double rowdd = (180/M_PI)*3600;

};


#endif
