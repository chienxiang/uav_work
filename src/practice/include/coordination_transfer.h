#include <iostream>
#include <string> 
#include <bits/stdc++.h> 
#include <math.h>
#include <cmath>
#include <complex>
#define CONSTANTS_RADIUS_OF_EARTH 6371000    //meters (m)
using namespace std;

class coordination_transfer
{
public:
    float lat_ref,lon_ref,lat,lon,local_x,local_y;
    float degrees;
    float GPS2XY(float lat_ref,float lon_ref,float lat,float lon)
    {
        //input GPS and Reference GPS in degrees
        //output XY in meters (m) X:North Y:East
        float lat_rad,lat_rad_ref,sin_lat,cos_lat,sin_lat_ref,cos_lat_ref;
        float lon_rad,lon_rad_ref,cos_d_lon;
        float arg,c,k;
        lat_rad = degreesToRadians(lat);
        lon_rad = degreesToRadians(lon);
        lat_rad_ref = degreesToRadians(lat_ref);
        lon_rad_ref = degreesToRadians(lon_ref);
        sin_lat = sin(lat_rad);
        cos_lat = cos(lat_rad);
        sin_lat_ref = sin(lat_rad_ref);
        cos_lat_ref = cos(lat_rad_ref);
        cos_d_lon = cos(lon_rad - lon_rad_ref);

        arg = clip(sin_lat_ref * sin_lat + cos_lat_ref * cos_lat * cos_d_lon, -1.0, 1.0);
        c = acos(arg);
        k=1.0;
        if(abs(c)>0)
        {
            k = (c/sin(c));
        }

        local_x = k * (cos_lat_ref * sin_lat - sin_lat_ref * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH; //x方向是南北
        local_y = k * cos_lat * sin(lon_rad - lon_rad_ref) * CONSTANTS_RADIUS_OF_EARTH;//y方向是東西

        return local_x, local_y;

    }
    float degreesToRadians(float degrees)
    {
        return degrees * M_PI / 180;
    }
    float clip(float n, float lower, float upper )
    {
        n = ( n > lower ) * n + !( n > lower ) * lower;
        return ( n < upper ) * n + !( n < upper ) * upper;
    }
};