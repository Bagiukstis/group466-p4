#ifndef JACO_CONTROL
#define JACO_CONTROL

struct bottle {
  float x;
  float y;
  float z;
};
struct cup{
  float x;
  float y;
  float z;
};

struct quaternions{
  float w;
  float x;
  float y;
  float z;
  float Y;          //YAW = Z
  float P;          //PITCH= Y
  float R;          //ROW - X
  float ang;
  float ang2;
  float hyp;
  float cy;
  float sy;
  float cp;
  float sp;
  float cr;
  float sr;
  const double d2r = 0.01745329251;
};
struct pouring{
  float x;
  float y;
  float z;
};
struct retract{
  float x;
  float y;
  float z;
};
struct approach{
  float x;
  float y;
  float z;
};

#endif
