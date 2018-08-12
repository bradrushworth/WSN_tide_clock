//////////////////////////////////////////////////////////////////////////////
//
// Moon phases based on algorithm published by Fraser Farrell 
// http://www.delphigroups.info/2/03/20272.html
//
/////////////////////////////////////////////////////////////////////////////////
//
// Moonrise / moonset algorithm by Stephen R. Schmitt 
// http://mysite.verizon.net/res148h4j/javascript/script_moon_rise_set2.html
//
//////////////////////////////////////////////////////////////////////////////////

/* Useful Constants */
#define SECS_PER_MIN  (60UL)
#define SECS_PER_HOUR (3600UL)
#define SECS_PER_DAY  (SECS_PER_HOUR * 24UL)
#define DAYS_PER_WEEK (7UL)
#define SECS_PER_WEEK (SECS_PER_DAY * DAYS_PER_WEEK)
#define SECS_PER_YEAR (SECS_PER_WEEK * 52UL)
#define SECS_YR_2000  (946684800UL) // the time at the start of y2k

struct moon_t
{
  byte light;
  byte phase;
  boolean isRise;
  boolean isSet;
  byte riseH;
  byte riseM;
  byte setH;
  byte setM;
} Moon, tmpMoon;

float moon_out;
float moon_age;
int prevNextDays[3] = {0, 0, 0};

int zone;
float Rise_az = 0.0;
float Set_az  = 0.0;
float mp[3][3];
float Sky[] = {0.0, 0.0, 0.0};
float RAn[] = {0.0, 0.0, 0.0};
float Dec[] = {0.0, 0.0, 0.0};
float VHz[] = {0.0, 0.0, 0.0};
const float DR = PI/180;
const float K1 = 15*DR*1.0027379;


int cdn(uint32_t t) {
  t -= 946684800; // Number of seconds between 1970-01-01 and 2000-01-01
  return t/SECS_PER_DAY;
}

float unwindangle(float a)      //converts very large angles to the range 0-360 degrees
{
  while (a>=360)
    a = a-360;
  while (a<0)
    a = a+360;
  return a;
}

float degtorad(float d)       //degrees to radians conversion
{
  d = d*PI/180;
  return d;
}

float radtodeg(float d)        //radians to decimal degrees conversion
{
  d = d*180/PI;
  return d; 
}

float moon_mean_longitude(float T1)
{
  float mml;
  mml = degtorad(unwindangle(218.3164591 
         +T1*(481267.88134236 +T1*(-1.3268e-3
         +T1*(1/538841-T1/65194000)))));
  return mml;
}

float moon_mean_anomaly(float T1)
{
  float mma;
  mma = degtorad(unwindangle(134.9634114 
         +T1*(477198.8676313 +T1*(8.997e-3 
         +T1*(1/69699-T1/14712000))))); 
  return mma;
}

float moon_mean_elongation(float T1)
{
  float mme;
  mme = degtorad(unwindangle(297.8502042 
         +T1*(445267.1115168 +T1*(-1.63e-3 
         +T1*(1/545868-T1/113065000))))); 
  return mme;
}

float sunanomaly(float T1)
{
  float sa, M, C;
  M = 357.5291 + T1*(35999.0503 - T1*(1.559e-4 + T1*4.8e-7)); 
  M = degtorad(M);
  C = sin(M)*(1.9146-T1*(0.004817+T1*1.4e-5)) 
       + sin(2*M)*(0.019993-T1*1.01e-4) 
       + sin(3*M)*2.9e-4; 
  sa = degtorad(unwindangle(C+357.5291 
              +T1*(35999.0503-T1*(1.559e-4 +T1/24490000)))); 
  return sa;
}

void moonage()
{  
  float tmpT = cdn(RTC.now().unixtime());
  float T = tmpT / 36525;             

  float phase_angle = 180
                   - radtodeg(moon_mean_elongation(T))
                   - 6.289*sin(moon_mean_anomaly(T)) 
                   + 2.100*sin(sunanomaly(T)) 
                   - 1.274*sin(2*moon_mean_elongation(T)
                   - moon_mean_anomaly(T))
                   - 0.658*sin(2*moon_mean_elongation(T))
                   - 0.214*sin(2*moon_mean_anomaly(T)) 
                   - 0.110*sin(moon_mean_elongation(T)); 
  phase_angle = degtorad(phase_angle);
  //moon_light = floor(100 * (1+cos(phase_angle))/2);
  moon_out = (100 * (1+cos(phase_angle))/2);
  moon_age = 29.530588*moon_mean_elongation(T)/(2*PI);
}




//////////////////////// Moonrise and Moonset /////////////////////////////////////////
float lst(int lon, int jd, float z ) 
{
    float s = 24110.5 + 8640184.812999999*jd/36525 + 86636.6*z + 86400*lon;
    s = s/86400;
    s = s - floor(s);
    return s*360*DR;
}

void moon(float jd)
{
  float d, f, g, h, m, n, s, u, v, w;
  
  h = 0.606434 + 0.03660110129*jd;
  m = 0.374897 + 0.03629164709*jd;
  f = 0.259091 + 0.0367481952 *jd;
  d = 0.827362 + 0.03386319198*jd;
  n = 0.347343 - 0.00014709391*jd;
  g = 0.993126 + 0.0027377785 *jd;
  
  h = h - floor(h);
  m = m - floor(m);
  f = f - floor(f);
  d = d - floor(d);
  n = n - floor(n);
  g = g - floor(g);

  h = h*2*PI;
  m = m*2*PI;
  f = f*2*PI;
  d = d*2*PI;
  n = n*2*PI;
  g = g*2*PI;

  v = 0.39558 * sin(f + n);
  v = v + 0.082 * sin(f);
  v = v + 0.03257 * sin(m - f - n);
  v = v + 0.01092 * sin(m + f + n);
  v = v + 0.00666 * sin(m - f);
  v = v - 0.00644 * sin(m + f - 2*d + n);
  v = v - 0.00331 * sin(f - 2*d + n);
  v = v - 0.00304 * sin(f - 2*d);
  v = v - 0.0024 * sin(m - f - 2*d - n);
  v = v + 0.00226 * sin(m + f);
  v = v - 0.00108 * sin(m + f - 2*d);
  v = v - 0.00079 * sin(f - n);
  v = v + 0.00078 * sin(f + 2*d + n);

  u = 1 - 0.10828 * cos(m);
  u = u - 0.0188 * cos(m - 2*d);
  u = u - 0.01479 * cos(2*d);
  u = u + 0.00181 * cos(2*m - 2*d);
  u = u - 0.00147 * cos(2*m);
  u = u - 0.00105 * cos(2*d - g);
  u = u - 0.00075 * cos(m - 2*d + g);

  w = 0.10478 * sin(m);
  w = w - 0.04105 * sin(2*f + 2*n);
  w = w - 0.0213 * sin(m - 2*d);
  w = w - 0.01779 * sin(2*f + n);
  w = w + 0.01774 * sin(n);
  w = w + 0.00987 * sin(2*d);
  w = w - 0.00338 * sin(m - 2*f - 2*n);
  w = w - 0.00309 * sin(g);
  w = w - 0.0019 * sin(2*f);
  w = w - 0.00144 * sin(m + n);
  w = w - 0.00144 * sin(m - 2*f - n);
  w = w - 0.00113 * sin(m + 2*f + 2*n);
  w = w - 0.00094 * sin(m - 2*d + g);
  w = w - 0.00092 * sin(2*m - 2*d);

    s = w/sqrt(u - v*v);                  // compute moon's right ascension ...  
    Sky[0] = h + atan(s/sqrt(1 - s*s));
    
    s = v/sqrt(u);                        // declination ...
    Sky[1] = atan(s/sqrt(1 - s*s));

    Sky[2] = 60.40974 * sqrt( u );          // and parallax
}

float interpolate(float f0, float f1, float f2, float p)
{
    float a = f1 - f0;
    float b = f2 - f1 - a;
    float f = f0 + p*(2*a + b*(2*p - 1));
    return f;
}

int sgn(float x )
{
    int rv;

    if (x > 0.0)      rv =  1;
    else if (x < 0.0) rv = -1;
    else              rv =  0;
    return rv;
}

float test_moon(int k, int zone, float t0, int lat, float plx )
{
    float ha[] = {0.0, 0.0, 0.0};
    float a, b, c, d, e, s, z;
    int hr, min; 
    float time;
    float az, hz, nz, dz;

    if (RAn[2] < RAn[0])
        RAn[2] = RAn[2] + 2*PI;

    ha[0] = t0 - RAn[0] + k*K1;
    ha[2] = t0 - RAn[2] + k*K1 + K1;

    ha[1]  = (ha[2] + ha[0])/2;                // hour angle at half hour
    Dec[1] = (Dec[2] + Dec[0])/2;              // declination at half hour

    s = sin(DR*lat);
    c = cos(DR*lat);

    // refraction + sun semidiameter at horizon + parallax correction
    z = cos(DR*(90.567 - 41.685/plx));

    if (k <= 0)                                // first call of function
       VHz[0] = s*sin(Dec[0]) + c*cos(Dec[0])*cos(ha[0]) - z;

    VHz[2] = s*sin(Dec[2]) + c*cos(Dec[2])*cos(ha[2]) - z;

    if (sgn(VHz[0]) == sgn(VHz[2]))
        return VHz[2];                         // no event this hour

    VHz[1] = s*sin(Dec[1]) + c*cos(Dec[1])*cos(ha[1]) - z;

    a = 2*VHz[2] - 4*VHz[1] + 2*VHz[0];
    b = 4*VHz[1] - 3*VHz[0] - VHz[2];
    d = b*b - 4*a*VHz[0];

    if (d < 0)
        return VHz[2];                         // no event this hour

    d = sqrt(d);
    e = (-b + d)/(2*a);

    if (( e > 1 )||( e < 0 ))
        e = (-b - d)/(2*a);

    time = k + e + 1/120;                      // time of an event + round up
    hr = floor(time);
    min = floor((time - hr)*60);

    hz = ha[0] + e*(ha[2] - ha[0]);            // azimuth of the moon at the event
    nz = -cos(Dec[1])*sin(hz);
    dz = c*sin(Dec[1]) - s*cos(Dec[1])*cos(hz);
    az = atan2(nz, dz)/DR;
    if (az < 0) az = az + 360;

    if ((VHz[0] < 0)&&(VHz[2] > 0))
      {
        Moon.riseH = hr;
        Moon.riseM = min;
        Rise_az = az;
        Moon.isRise = true;
      }

    if ((VHz[0] > 0)&&(VHz[2] < 0))
      {
        Moon.setH = hr;
        Moon.setM = min;
        Set_az = az;
        Moon.isSet = true;
      }
      
    return VHz[2];
}

void riseset(int lat, int lon, boolean next_day=0)    ///////////////// main function ////////////////////
{  
  float jd;
  
  Moon.riseH = 0;
  Moon.riseM = 0;
  Moon.setH = 0;
  Moon.setM = 0;
  
  if (next_day) 
    jd = cdn(RTC.now().unixtime()) +1;
  else 
    jd = cdn(RTC.now().unixtime());
  
  for (int i=0; i<3; i++) {
    for (int j=0; j<3; j++) {
     mp[i][j] = 0.0;
    }
  }
  
  int x = lon;
  zone = -x/15;
  
  lon = lon/360;
  float tz = zone/24;
  float t0 = lst(lon, jd, tz);
  jd = jd + tz;
  
  for (int k=0; k<3; k++)
   {
    moon(jd);
    mp[k][0] = Sky[0];
    mp[k][1] = Sky[1];
    mp[k][2] = Sky[2];
    jd = jd +0.5;
   }
  
  if (mp[1][0] <= mp[0][0])
     mp[1][0] = mp[1][0] + 2*PI;

  if (mp[2][0] <= mp[1][0])
     mp[2][0] = mp[2][0] + 2*PI;
  
  RAn[0] = mp[0][0];
  Dec[0] = mp[0][1];

  Moon.isRise = false;                          // initialize
  Moon.isSet  = false;

  for (int k=0; k<24; k++)
   {
    float ph = (k + 1)/24;
     
    RAn[2] = interpolate(mp[0][0], mp[1][0], mp[2][0], ph);
    Dec[2] = interpolate(mp[0][1], mp[1][1], mp[2][1], ph);
    
    VHz[2] = test_moon(k, zone, t0, lat, mp[1][2]);
    
    RAn[0] = RAn[2];                       // advance to next hour
    Dec[0] = Dec[2];
    VHz[0] = VHz[2];
   }
}
