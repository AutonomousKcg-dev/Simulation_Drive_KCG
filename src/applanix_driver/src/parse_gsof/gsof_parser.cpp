/* Copyright(c) 2005 Trimble Navigation Ltd.
 * $Id: gsofParser.c,v 1.7 2011/11/15 20:37:11 tom Exp $
 * $Source: /home/CVS/panem/bin/src/gsofParser/gsofParser.c,v $
 *
 * gsofParser.c
 *
 * Parses stdin and extracts/lists GSOF information from it.
 *
 * The data is assumed to be the raw GSOF output from a Trimble receiver.
 * That is, it consists of Trimcomm packets (02..03) of type 0x40 in which
 * are embedded GSOF subtype records.  The program accepts such data on
 * standard input (either live as part of a '|'-pipeline, or from a file
 * via '<'-redirection.  It synchronizes with the individual Trimcomm packets
 * and extracts the contents.  When a complete set of GSOF-0x40 packets is
 * collected, the total contents is parsed and listed.  For some GSOF subtypes
 * there is a full decoder below and the contents will be listed, item by item.
 * Other packets  will just be listed as Hex bytes.  You can write additional
 * Routines to the decoder should you need to, using the Routines as models to
 * implement for additional GSOF subtypes.
 *
 * The program starts with main which collects Trimcomm packets.  Then moves to
 * postGsofData() which collects the GSOF data from multiple packets and decides
 * when a complete set has been received.  Then it goes to processGsofData() which
 * steps through the collected data parsing the individual gsof subtype records.
 * If the GSOF subtype is one of the special ones where it has a decoder, that
 * decoder is called, otherwise program just dumps the Hex bytes of the record.
 * The program runs until the Stdinput indicates end of file (EOF)  [see gc()] or
 * the user stops it with a "Ctrl" plus "C") action.
 *
 * NOTE: This program isn't designed to handle corrupted data.  There isn�t any
 * sophisticated logic to handle corrupted data packets.  This source is being
 * provided "As Is".  Program was written with the idea to view the contents of
 * well-formed GSOF data, not to debug the overall formatting.  There should be
 * some resistant to additional data such as NMEA being mixed into the GSOF stream,
 * this has not been validated.
 */

#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <parse_gsof/gsofParser.hpp>

#define PI (3.14159F)




/**********************************************************************/
unsigned long getU32( unsigned char * * ppData )
/**********************************************************************/
// Used by the decoding routines to grab 4 bytes and pack them into
// a U32.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
// This is designed to work on little-endian processors (Like Pentiums).
// Effectively that means we reverse the order of the bytes.
// This would need to be rewritten to work on big-endian PowerPCs.
{
  unsigned long retValue ;
  unsigned char * pBytes ;

  pBytes = (unsigned char *)(&retValue) + 3 ;

  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes   = *(*ppData)++ ;

  return retValue ;

} /* end of getU32() */



/**********************************************************************/
float getFloat( unsigned char * * ppData )
/**********************************************************************/
// Used by the decoding routines to grab 4 bytes and pack them into
// a Float.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
// This is designed to work on little-endian processors (Like Pentiums).
// Effectively that means we reverse the order of the bytes.
// This would need to be rewritten to work on big-endian PowerPCs.
{
  float retValue ;
  unsigned char * pBytes ;

  pBytes = (unsigned char *)(&retValue) + 3 ;


  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes   = *(*ppData)++ ;

  return retValue ;

} /* end of getFloat() */



/**********************************************************************/
double getDouble( unsigned char * * ppData )
/**********************************************************************/
// Used by the decoding routines to grab 8 bytes and pack them into
// a Double.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
// This is designed to work on little-endian processors (Like Pentiums).
// Effectively that means we reverse the order of the bytes.
// This would need to be rewritten to work on big-endian PowerPCs.
{
  double retValue ;
  unsigned char * pBytes ;

  pBytes = (unsigned char *)(&retValue) + 7 ;


  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes-- = *(*ppData)++ ;
  *pBytes   = *(*ppData)++ ;

  return retValue ;

} /* end of getDouble() */



/**********************************************************************/
unsigned short getU16( unsigned char * * ppData )
/**********************************************************************/
// Used by the decoding routines to grab 2 bytes and pack them into
// a U16.  Fed ppData which is a pointer to a pointer to the start of
// the data bytes.  The pointer variable referenced by ppData is moved
// beyond the four bytes.
// This is designed to work on little-endian processors (Like Pentiums).
// Effectively that means we reverse the order of the bytes.
// This would need to be rewritten to work on big-endian PowerPCs.
{
  unsigned short retValue ;
  unsigned char * pBytes ;

  pBytes = (unsigned char *)(&retValue) + 1 ;

  *pBytes-- = *(*ppData)++ ;
  *pBytes   = *(*ppData)++ ;

  return retValue ;

} /* end of getU16() */


/***********************************************************************
 * The next section contains routines which are parsers for individual
 * GSOF records.  They are all passed a length (which is listed but
 * usually not used) and a pointer to the data bytes that make up the
 * record.
 ***********************************************************************
 */


/**********************************************************************/
void processPositionTime( int length, unsigned char *pData )
/**********************************************************************/
{
  unsigned long msecs ;
  unsigned short weekNumber ;
  int nSVs ;
  int flags1 ;
  int flags2 ;
  int initNumber ;

  printf( "  GsofType:1 - PositionTime  len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  msecs = getU32( &pData ) ;
  weekNumber = getU16( &pData ) ;
  nSVs = *pData++ ;
  flags1 = *pData++ ;
  flags2 = *pData++ ;
  initNumber = *pData++ ;

  printf( "  Milliseconds:%ld  Week:%d  #Svs:%d "
          "flags:%02X:%02X init:%d\n",
          msecs,
          weekNumber,
          nSVs,
          flags1,
          flags2,
          initNumber
        ) ;


} /* end of processPositionTime() */





/**********************************************************************/
void processLatLonHeight( int length, unsigned char *pData )
/**********************************************************************/
{
  double lat, lon, height ;

  printf( "  GsofType:2 - LatLongHeight   len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif
  lat = getDouble( &pData ) * 180.0 / PI ;
  lon = getDouble( &pData ) * 180.0 / PI ;
  height = getDouble( &pData ) ;

  printf( "  Lat:%.7f Lon:%.7f Height:%.3f\n",
          lat,
          lon,
          height
        ) ;
} /* end of processLatLonHeight() */





/**********************************************************************/
void processECEF( int length, unsigned char *pData )
/**********************************************************************/
{
  double X, Y, Z ;

  printf( "  GsofType:3 - ECEF   len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif
  X = getDouble( &pData ) ;
  Y = getDouble( &pData ) ;
  Z = getDouble( &pData ) ;

  printf( "  X:%.3f Y:%.3f Z:%.3f\n", X, Y, Z ) ;

} /* end of processECEF() */



/**********************************************************************/
void processLocalDatum( int length, unsigned char *pData )
/**********************************************************************/
{
  char id[9] ;
  double lat, lon, height ;

  printf( "  GsofType:4 - Local Datum Position  "
          "!!!!!UNTESTED!!!!!!!  len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  memcpy( id, pData, 8 ) ;
  pData += 8 ;
  // id[9] = 0 ;//error - char id[9]

  lat = getDouble( &pData ) * 180.0 / PI ;
  lon = getDouble( &pData ) * 180.0 / PI ;
  height = getDouble( &pData ) ;

  printf( "  Id:%s Lat:%.7f Lon:%.7f Height:%.3f\n",
          id,
          lat,
          lon,
          height
        ) ;
} /* end of processLocalDatum() */



/**********************************************************************/
void processEcefDelta( int length, unsigned char *pData )
/**********************************************************************/
{
  double X, Y, Z ;

  printf( "  GsofType:6 - ECEF Delta  len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  X = getDouble( &pData ) ;
  Y = getDouble( &pData ) ;
  Z = getDouble( &pData ) ;

  printf( "  X:%.3f Y:%.3f Z:%.3f\n", X, Y, Z ) ;

} /* end of processEcefDelta() */



/**********************************************************************/
void processTangentPlaneDelta( int length, unsigned char *pData )
/**********************************************************************/
{
  double E, N, U ;

  printf( "  GsofType:7 - Tangent Plane Delta  len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  E = getDouble( &pData ) ;
  N = getDouble( &pData ) ;
  U = getDouble( &pData ) ;

  printf( "  East:%.3f North:%.3f Up:%.3f\n", E, N, U ) ;

} /* end of processTangentPlaneDelta() */



/**********************************************************************/
void processVelocityData( int length, unsigned char *pData )
/**********************************************************************/
{
  int flags ;
  float velocity ;
  float heading ;
  float vertical ;

  printf( "  GsofType:8 - Velocity Data  len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  flags = *pData++ ;

  velocity = getFloat( &pData ) ;
  heading = getFloat( &pData ) * 180.0F / PI ;
  vertical = getFloat( &pData ) ;

  printf( "  Flags:%02X  velocity:%.3f  heading:%.3f  vertical:%.3f\n",
          flags,
          velocity,
          heading,
          vertical
        ) ;

} /* end of processVelocityData() */



/**********************************************************************/
void processUtcTime( int length, unsigned char *pData )
/**********************************************************************/
{

  printf( "  GsofType:16 - UTC Time Info   len:%d\n",
          length
        ) ;

  U32 msecs = getU32( &pData ) ;
  U16 weekNumber = getU16( &pData ) ;
  S16 utcOffset = getU16( &pData ) ;
  U8 flags = *pData++ ;

  printf( "  ms:%lu  week:%u  utcOff:%d  flags:%02x\n",
          msecs,
          weekNumber,
          utcOffset,
          flags
        ) ;

} /* end of processUtcTime() */



/**********************************************************************/
void processPdopInfo( int length, unsigned char *pData )
/**********************************************************************/
{
  float pdop ;
  float hdop ;
  float vdop ;
  float tdop ;

  printf( "  GsofType:9 - PDOP Info   len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  pdop = getFloat( &pData ) ;
  hdop = getFloat( &pData ) ;
  vdop = getFloat( &pData ) ;
  tdop = getFloat( &pData ) ;

  printf( "  PDOP:%.1f  HDOP:%.1f  VDOP:%.1f  TDOP:%.1f\n",
          pdop,
          hdop,
          vdop,
          tdop
        ) ;

} /* end of processPdopInfo() */



/**********************************************************************/
void processBriefSVInfo( int length, unsigned char *pData )
/**********************************************************************/
{
  int nSVs ;
  int i ;

  printf( "  GsofType:13 - SV Brief Info   len:%d\n",
          length
        ) ;

  nSVs = *pData++ ;
  printf( "  SvCount:%d\n", nSVs ) ;

  for ( i = 0 ; i < nSVs ; ++i )
  {
    int prn ;
    int flags1 ;
    int flags2 ;

    prn = *pData++ ;
    flags1 = *pData++ ;
    flags2 = *pData++ ;

    printf( "  Prn:%-2d  flags:%02X:%02X\n", prn, flags1, flags2 );
  }
} /* end of processBriefSVInfo */



/**********************************************************************/
void processAllBriefSVInfo( int length, unsigned char *pData )
/**********************************************************************/
{
  int nSVs ;
  int i ;

  printf( "  GsofType:33 - All SV Brief Info   len:%d\n",
          length
        ) ;

  nSVs = *pData++ ;
  printf( "  SvCount:%d\n", nSVs ) ;

  for ( i = 0 ; i < nSVs ; ++i )
  {
    int prn ;
    int system ;
    int flags1 ;
    int flags2 ;

    prn = *pData++ ;
    system = *pData++;
    flags1 = *pData++ ;
    flags2 = *pData++ ;

    printf( "  %s SV:%-2d  flags:%02X:%02X\n",
            system == 0 ? "GPS"
            : system == 1 ? "SBAS"
            : system == 2 ? "GLONASS"
            : system == 3 ? "GALILEO"
	    : system == 4 ? "QZSS"
            : system == 5 ? "BEIDOU"
            : system == 6 ? "RESERVED" : "RESERVED",
            prn, flags1, flags2 );
  }
} /* end of processAllBriefSVInfo */



/**********************************************************************/
void processAllDetailedSVInfo( int length, unsigned char *pData )
/**********************************************************************/
{
  int nSVs ;
  int i ;

  printf( "  GsofType:34 - All SV Detailed Info   len:%d\n",
          length
        ) ;

  nSVs = *pData++ ;
   printf( "  SvCount:%d\n", nSVs ) ;

  for ( i = 0 ; i < nSVs ; ++i )
  {
    int prn ;
    int system ;
    int flags1 ;
    int flags2 ;
    int elevation ;
    int azimuth ;
    int snr[ 3 ];

    prn = *pData++ ;
    system = *pData++;
    flags1 = *pData++ ;
    flags2 = *pData++ ;
    elevation = *pData++ ;
    azimuth = getU16( &pData ) ;
    snr[ 0 ] = *pData++;
    snr[ 1 ] = *pData++;
    snr[ 2 ] = *pData++;

    #if 0
    {
    	int ii ;
    	for ( ii = 0 ; ii < length ; ++ii )
    	{
      		printf( "%02X%c", pData[ii], ii % 16 == 15 ? '\n' : ' ') ;
    	}
    	printf( "\n" ) ;
    }
    #endif

    printf( "  %s SV:%-2d  flags:%02X:%02X\n"
            "     El:%2d  Az:%3d\n"
            "     SNR %3s %5.2f\n"
            "     SNR %3s %5.2f\n"
            "     SNR %3s %5.2f\n",
            system == 0 ? "GPS"
            : system == 1 ? "SBAS"
            : system == 2 ? "GLONASS"
            : system == 3 ? "GALILEO"
	    : system == 4 ? "QZSS"
            : system == 5 ? "BEIDOU"
            : system == 6 ? "RESERVED" : "RESERVED",
            prn, flags1, flags2,
            elevation, azimuth,
            system == 3 ? "E1 " : "L1 ", (float)snr[ 0 ] / 4.0,
            system == 3 ? "N/A " : "L2 ", (float)snr[ 1 ] / 4.0,
            system == 3 ? "E5 "
              : system == 2 ? "G1P" : "L5 ", (float)snr[ 2 ] / 4.0
          );

  }

} /* end of processAllDetailedSVInfo */



/**********************************************************************/
void processSvDetailedInfo( int length, unsigned char *pData )
/**********************************************************************/
{
  int nSVs ;
  int i ;

  printf( "  GsofType:14 - SV Detailed Info   len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  nSVs = *pData++ ;
  printf( "  SvCount:%d\n", nSVs ) ;

  for ( i = 0 ; i < nSVs ; ++i )
  {
    int prn ;
    int flags1 ;
    int flags2 ;
    int elevation ;
    int azimuth ;
    int l1Snr ;
    int l2Snr ;

    prn = *pData++ ;
    flags1 = *pData++ ;
    flags2 = *pData++ ;
    elevation = *pData++ ;
    azimuth = getU16( &pData ) ;
    l1Snr = *pData++ ;
    l2Snr = *pData++ ;

    printf( "   Prn:%-2d  flags:%02X:%02X elv:%-2d azm:%-3d  "
            "L1snr:%-5.2f L2snr:%-5.2f\n",
            prn,
            flags1,
            flags2,
            elevation,
            azimuth,
            ((double)l1Snr) / 4.0 ,
            ((double)l2Snr) / 4.0
          ) ;
  }
} /* end of processSvDetailedInfo() */



/**********************************************************************/
void processAttitudeInfo( int length , unsigned char *pData )
/**********************************************************************/
{
  double gpsTime ;
  unsigned char flags ;
  unsigned char nSVs ;
  unsigned char mode ;
  double pitch ;
  double yaw ;
  double roll ;
  double range ;
  double pdop ;

  printf( "  GsofType:27 - AttitudeInfo  len:%d\n",
          length
        ) ;

  #if 0
  {
    int i ;
    for ( i = 0 ; i < length ; ++i )
    {
      printf( "%02X%c",
              pData[i],
              i % 16 == 15 ? '\n' : ' '
            ) ;
    }
    printf( "\n" ) ;
  }
  #endif

  gpsTime = (double)getU32( &pData ) / 1000.0 ;
  flags = *pData++ ;
  nSVs = *pData++ ;
  mode = *pData++ ;
  ++pData ; // reserved
  pitch = getDouble( &pData ) / PI * 180.0 ;
  yaw   = getDouble( &pData ) / PI * 180.0 ;
  roll  = getDouble( &pData ) / PI * 180.0 ;
  range = getDouble( &pData ) ;

  pdop  = (double)getU16( &pData ) / 10.0 ;

  printf( "  Time:%.3f"
          " flags:%02X"
          " nSVs:%d"
          " mode:%d\n"
          "  pitch:%.3f"
          " yaw:%.3f"
          " roll:%.3f"
          " range:%.3f"
          " pdop:%.1f"
          "\n",
          gpsTime,
          flags,
          nSVs,
          mode,
          pitch,
          yaw,
          roll,
          range,
          pdop
        ) ;

  // Detect if the extended record information is present
  if ( length > 42 )
  {
    float pitch_var ;
    float yaw_var ;
    float roll_var ;
    float pitch_yaw_covar ;
    float pitch_roll_covar ;
    float yaw_roll_covar ;
    float range_var;

    // The variances are in units of radians^2
    pitch_var = getFloat( &pData ) ;
    yaw_var   = getFloat( &pData ) ;
    roll_var  = getFloat( &pData ) ;

    // The covariances are in units of radians^2
    pitch_yaw_covar  = getFloat( &pData ) ;
    pitch_roll_covar = getFloat( &pData ) ;
    yaw_roll_covar   = getFloat( &pData ) ;

    // The range variance is in units of m^2
    range_var = getFloat( &pData ) ;

    printf( "  variance (radians^2)"
            " pitch:%.4e"
            " yaw:%.4e"
            " roll:%.4e"
            "\n",
            pitch_var,
            yaw_var,
            roll_var ) ;

    printf( "  covariance (radians^2)"
            " pitch-yaw:%.4e"
            " pitch-roll:%.4e"
            " yaw-roll:%.4e"
            "\n",
            pitch_yaw_covar,
            pitch_roll_covar,
            yaw_roll_covar ) ;

    printf( "  variance (m^2)"
            " range: %.4e"
            "\n",
            range_var ) ;
  }

} /* end of processAttitudeInfo() */


/**********************************************************************/
void processLbandStatus( int length , unsigned char *pData )
/**********************************************************************/
{
  unsigned char name[5];
  float freq;
  unsigned short bit_rate;
  float snr;
  unsigned char hp_xp_subscribed_engine;
  unsigned char hp_xp_library_mode;
  unsigned char vbs_library_mode;
  unsigned char beam_mode;
  unsigned char omnistar_motion;
  float horiz_prec_thresh;
  float vert_prec_thresh;
  unsigned char nmea_encryption;
  float iq_ratio;
  float est_ber;
  unsigned long total_uw;
  unsigned long total_bad_uw;
  unsigned long total_bad_uw_bits;
  unsigned long total_viterbi;
  unsigned long total_bad_viterbi;
  unsigned long total_bad_messages;
  unsigned char meas_freq_is_valid = -1;
  double        meas_freq = 0.0;

  printf( "  GsofType:40 - LBAND status  len:%d\n",
          length
        ) ;

  memcpy( name, pData, 5 );
  pData += 5;
  freq = getFloat( &pData );
  bit_rate = getU16( &pData );
  snr = getFloat( &pData );
  hp_xp_subscribed_engine = *pData++;
  hp_xp_library_mode = *pData++;
  vbs_library_mode = *pData++;
  beam_mode = *pData++;
  omnistar_motion = *pData++;
  horiz_prec_thresh = getFloat( &pData );
  vert_prec_thresh = getFloat( &pData );
  nmea_encryption = *pData++;
  iq_ratio = getFloat( &pData );
  est_ber = getFloat( &pData );
  total_uw = getU32( &pData );
  total_bad_uw = getU32( &pData );
  total_bad_uw_bits = getU32( &pData );
  total_viterbi = getU32( &pData );
  total_bad_viterbi = getU32( &pData );
  total_bad_messages = getU32( &pData );
  if( length > 61 )
  {
    meas_freq_is_valid = *pData++;
    meas_freq = getDouble( &pData );
  }

  printf( "  Name:%s"
          "  Freq:%g"
          "  bit rate:%d"
          "  SNR:%g"
          "\n"
          "  HP/XP engine:%d"
          "  HP/XP mode:%d"
          "  VBS mode:%d"
          "\n"
          "  Beam mode:%d"
          "  Omnistar Motion:%d"
          "\n"
          "  Horiz prec. thresh.:%g"
          "  Vert prec. thresh.:%g"
          "\n"
          "  NMEA encryp.:%d"
          "  I/Q ratio:%g"
          "  Estimated BER:%g"
          "\n"
          "  Total unique words(UW):%d"
          "  Bad UW:%d"
          "  Bad UW bits:%d"
          "\n"
          "  Total Viterbi:%d"
          "  Corrected Viterbi:%d"
          "  Bad messages:%d"
          "\n"
          "  Meas freq valid?:%d"
          "  Meas freq:%.3f"
          "\n"
          ,
          name,
          freq,
          bit_rate,
          snr,
          hp_xp_subscribed_engine,
          hp_xp_library_mode,
          vbs_library_mode,
          beam_mode,
          omnistar_motion,
          horiz_prec_thresh,
          vert_prec_thresh,
          nmea_encryption,
          iq_ratio,
          est_ber,
          total_uw,
          total_bad_uw,
          total_bad_uw_bits,
          total_viterbi,
          total_bad_viterbi,
          total_bad_messages,
          meas_freq_is_valid,
          meas_freq
        ) ;

} /* end of processLbandStatus() */


/***********************************************************************
 * End of the GSOF subtype parsers.
 **********************************************************************
 */






gsofParser::gsofParser()
{

}
gsofParser::~gsofParser()
{
  
}

bool gsofParser::processInsFullNavigationInfo( int length, unsigned char *pData ,InsFullNavigationInfo & navInfo)
{
  // printf( "  GsofType:49 - InsFullNavigationInfo  len:%d\n",
  //         length
  //       ) ;

  #if 0
  {
    int i ;
    // for ( i = 0 ; i < length ; ++i )
    // {
    //   printf( "%02X%c",
    //           pData[i],
    //           i % 16 == 15 ? '\n' : ' '
    //         ) ;
    // }
    // printf( "\n" ) ;
  }
  #endif

    navInfo.GPS_Week_Number = getU16( &pData );
    navInfo.GPS_Time = getU32( &pData );

    navInfo.IMU_alignment_status = *pData++;
    navInfo.GPS_quality_indicator = *pData++;

    navInfo.Latitude = getDouble( &pData );
    navInfo.Longitude = getDouble( &pData );
    navInfo.Altitude = getDouble( &pData );

    navInfo.NorthVelocity = getFloat( &pData );
    navInfo.EastVelocity = getFloat( &pData );
    navInfo.DownVelocity = getFloat( &pData );
    navInfo.TotalSpeed = getFloat( &pData );
 
/*   
    navInfo.Roll = deg2rad(getDouble( &pData ) );
    navInfo.Pitch = deg2rad(getDouble( &pData ));
    navInfo.Heading = deg2rad(getDouble( &pData ));
    navInfo.TrackAngle = deg2rad(getDouble( &pData ));

    navInfo.AngularRate_x = deg2rad(getFloat( &pData ));
    navInfo.AngularRate_y = deg2rad(getFloat( &pData ));
    navInfo.AngularRate_z = deg2rad(getFloat( &pData ));
    */

    
    navInfo.Roll = getDouble( &pData );
    navInfo.Pitch =getDouble( &pData );
    navInfo.Heading = getDouble( &pData );
    navInfo.TrackAngle = getDouble( &pData );

    navInfo.AngularRate_x =  getFloat( &pData );
    navInfo.AngularRate_y = getFloat( &pData );
    navInfo.AngularRate_z = getFloat( &pData );

       

    navInfo.Longitudinal_acceleration = getFloat( &pData );
    navInfo.Traverse_acceleration = getFloat( &pData );
    navInfo.Down_acceleration = getFloat( &pData );





  return false;
}




/**********************************************************************/
int gsofParser::  processGsofData(unsigned char * gsofData,InsFullNavigationInfo & navInfo)
/**********************************************************************/
/* Called when a complete set of GSOF packets has been received.
 * The data bytes collected are available in global gsofData and the
 * number of those bytes is in gsofDataIndex.
 *
 * This routine just goes through the bytes and parses the sub-type
 * records.  Each of those has a Type and a Length.  If the type is
 * one of the special types we know about, we call the proper parser.
 * Otherwise we just hex-dump the record.
 */
{
  int i ;
  int gsofType ;
  int gsofLength ;
  unsigned char * pData ;

  bool full_info_in = false;
  //printf( "\nGSOF Records\n" ) ;
  pData = gsofData ;

  while (pData < gsofData + gsofDataIndex )
  {
    gsofType   = *pData++ ;
    gsofLength = *pData++ ;

    // If the type is one that we know about, then call the specific
    // parser for that type.
      // printf("gsofType: %d",gsofType);
      

    if (gsofType == 49)//InsFullNavigationInfo
    {
      // printf("gsofType == 49");
      full_info_in = processInsFullNavigationInfo(gsofLength, pData ,navInfo);
      pData += gsofLength ;
    }
    else
    if ( gsofType == 1 )
    {
      processPositionTime( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 2 )//2
    {
      processLatLonHeight( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 3 )
    {
      processECEF( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 4 )
    {
      processLocalDatum( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 8 )
    {
      processVelocityData( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 9 )
    {
      processPdopInfo( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 13 )
    {
      processBriefSVInfo( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 16 )
    {
      processUtcTime( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 33 )
    {
      processAllBriefSVInfo( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 34 )
    {
      processAllDetailedSVInfo( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 14 )
    {
      processSvDetailedInfo( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 27 )
    {
      processAttitudeInfo( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 6 )
    {
      processEcefDelta( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 7 )
    {
      processTangentPlaneDelta( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    if ( gsofType == 40 )
    {
      processLbandStatus( gsofLength, pData ) ;
      pData += gsofLength ;
    }
    else
    {
      // Not a type we know about.  Hex dump the bytes and move on.
      // printf( "  GsofType:%d  len:%d\n  ",
      //         gsofType,
      //         gsofLength
      //       ) ;

      // for ( i = 0 ; i < gsofLength ; ++i )
      // {
      //   printf( "%02X%s",
      //           *pData++,
      //           i % 16 == 15 ? "\n  " : " "
      //         ) ;
      // }
      // Terminate the last line if needed.
      // if (gsofLength %16 != 0)
      //   printf( "\n" ) ;
    }

    //printf( "\n" ) ;
  }
  //printf( "\n" ) ;

if(full_info_in)
  return 1;
return 0;
} /* end of processGsofData() */

// void convert_gsof_data(unsigned char * pData, int length, double * lat,double * lon)
// {

// }


/**********************************************************************/
int gsofParser::postGsofData( unsigned char * input_data,InsFullNavigationInfo & navInfo)
/**********************************************************************/
// Called whenever we get a new Trimcomm GSOF packet (type 0x40).
// These all contain a portion (or all) of a complete GSOF packet.
// Each portion contains a Transmission Number, an incrementing value
// linking related portions.
// Each portion contains a Page Index, 0..N, which increments for each
// portion in the full GSOF packet.
// Each portion contains a Max Page Index, N, which is the same for all
// portions.
//
// Each portion's data is appended to the global buffer, gsofData[].
// The next available index in that buffer is always gsofDataIndex.
// When we receive a portion with Page Index == 0, that signals the
// beginning of a new GSOF packet and we restart the gsofDataIndex at
// zero.
//
// When we receive a portion where Page Index == Max Page Index, then
// we have received the complete GSOF packet and can decode it.
{

  unsigned char * input_data_copy = input_data;
  int tcStx ;
  int tcStat ;
  int tcType ;
  int tcLength ;
  int tcCsum ;
  int tcEtx ;
  unsigned char *pData = new unsigned char[256];
  //*input_data_copy++;
  // printf("input_data_copy: ");
  // for(int i =0;input_data_copy[i]!='\0';i++)
  //   printf("%x  ", input_data_copy[i]);
   if ( *input_data_copy == 0x02 )
      {
      //  printf("0x02 \n");
        *input_data_copy++;
      }
    else
    {
    //  printf("0x02 not here\n");
      return -1;
    }
    tcStat = *input_data_copy++;
    // printf("tcStat: %x\n",tcStat);

    tcType = *input_data_copy++;
    // printf("tcType: %x\n",tcType);

    tcLength = *input_data_copy++;
    // printf("tcLength: %d\n",tcLength);

    for ( int i = 0 ; i < tcLength ; ++i )
      * (pData+i) = * (input_data_copy+i) ;



  int gsofTransmissionNumber ;
  int gsofPageIndex ;
  int gsofMaxPageIndex ;
  int i ;
      
  //
  gsofTransmissionNumber = *pData++ ;
  // printf("gsofTransmissionNumber: %d,\n",gsofTransmissionNumber);

  gsofPageIndex = *pData++ ;
  // printf("gsofPageIndex: %d, \n",gsofPageIndex);

  gsofMaxPageIndex = *pData++ ;
  // printf("gsofMaxPageIndex: %d\n", gsofMaxPageIndex);

// gsofMaxPageIndex = 0;//tmp
  // printf( "  GSOF packet: Trans#:%d  Page:%d MaxPage:%d\n",
  //         gsofTransmissionNumber,
  //         gsofPageIndex,
  //         gsofMaxPageIndex
  //       ) ;

  // If this is the first portion, restart the buffering system.
  if (gsofPageIndex == 0)
    gsofDataIndex = 0 ;

  // Transfer the data bytes in this portion to the global buffer.
  for (i = 3 ; i < tcLength ; ++i)
    gsofData[ gsofDataIndex++ ] = *pData++ ;

  // If this is the last portion in a packet, process the whole packet.

  if (gsofPageIndex == gsofMaxPageIndex)
  {
      // printf("processGsofData\n");
    
    return processGsofData(gsofData,navInfo);//return 1 if navInfo is updated
  }
  return 0;

} /* end of postGsofData() */


/**********************************************************************/
int gc( void )
/**********************************************************************/
/* This is a getchar() wrapper.  It just returns the characters
 * from standard input.  If it detects end of file, it aborts
 * the entire program.
 *
 * NOTE: This function is not optimal because if program is in the middle
 * of a packet there is no indication. This is a simple parsing application
 */
{
  int c ;

  c = getchar() ;
  if (c != EOF)
    return c ;

  printf( "END OF FILE \n" ) ;
  _exit( 0 ) ;
} /* end of gc() */



// /**********************************************************************/
// int main( int argn, char **argc )
// /**********************************************************************/
// /* Main entry point.  Looks for Trimcomm packets.  When we find one with
//  * type 0x40, its bytes are extracted and passed on to the GSOF
//  * handler.
//  */
// {
//   int tcStx ;
//   int tcStat ;
//   int tcType ;
//   int tcLength ;
//   int tcCsum ;
//   int tcEtx ;
//   unsigned char tcData[256] ;
//   int i ;


//   printf( "GSOF Parser\n") ;

//   tcLength = 10;
//   tcData[0] = 1;
//   tcData[1] = 1;
//   tcData[2] = 1;
//   tcData[3] = 1;
//   postGsofData( tcData, tcLength );

//   // while ( 1 )
//   // {
//   //   tcStx = gc() ;
//   //   if ( tcStx == 0x02 )
//   //   {
//   //     tcStat = gc() ;
//   //     tcType = gc() ;
//   //     tcLength = gc() ;
//   //     for ( i = 0 ; i < tcLength ; ++i )
//   //       tcData[i] = gc() ;

//   //     tcCsum = gc() ;
//   //     tcEtx = gc() ;
//   //     printf( "STX:%02Xh  Stat:%02Xh  Type:%02Xh  "
//   //             "Len:%d  CS:%02Xh  ETX:%02Xh\n",
//   //             tcStx,
//   //             tcStat,
//   //             tcType,
//   //             tcLength,
//   //             tcCsum,
//   //             tcEtx
//   //           ) ;

//   //     if (tcType == 0x40)
//   //       postGsofData( tcData, tcLength ) ;
//   //   }
//   //   else
//   //     printf( "Skipping %02X\n", tcStx ) ;
//   // }

//   return 0 ;
// } // main

