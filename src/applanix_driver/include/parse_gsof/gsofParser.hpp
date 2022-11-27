typedef unsigned long U32 ;
typedef unsigned short U16 ;
typedef signed short S16 ;
typedef unsigned char U8 ;

class InsFullNavigationInfo
{
public:
    U32 GPS_Week_Number;
    U32 GPS_Time;
    unsigned char IMU_alignment_status,GPS_quality_indicator;
    double Latitude, Longitude, Altitude;
    float NorthVelocity,EastVelocity,DownVelocity,TotalSpeed;
    double Roll,Pitch,Heading,TrackAngle;
    float AngularRate_x,AngularRate_y,AngularRate_z;
    float Longitudinal_acceleration,Traverse_acceleration,Down_acceleration;
};

class gsofParser
{
public:
  gsofParser();
  ~gsofParser();
  int postGsofData( unsigned char * input_data,InsFullNavigationInfo & navInfo);
private:
  unsigned char gsofData[2048] ;
  int gsofDataIndex ;

  bool processInsFullNavigationInfo( int length, unsigned char *pData ,InsFullNavigationInfo & navInfo);
  int processGsofData(unsigned char * gsofData,InsFullNavigationInfo & navInfo);

};
