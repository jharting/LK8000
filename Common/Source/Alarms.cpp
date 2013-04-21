/*
   LK8000 Tactical Flight Computer -  WWW.LK8000.IT
   Released under GNU/GPL License v.2
   See CREDITS.TXT file for authors and copyrights

   $Id$
*/

#include "externs.h"

// #define DEBUG_LKALARMS	1

void InitAlarms(void) {

  #if DEBUG_LKALARMS
  StartupStore(_T("...... Alarms: InitAlarms\n"));
  #endif
  int i;
  for (i=0; i<MAXLKALARMS; i++) {
	LKalarms[i].triggervalue=0;
	LKalarms[i].lastvalue=0;
	LKalarms[i].lasttriggertime=0.0;
	LKalarms[i].triggerscount=0;
  }
	/* Test values
	LKalarms[0].triggervalue=500;
	LKalarms[1].triggervalue=0;
	LKalarms[2].triggervalue=1200;
	*/

  for (int i = 0; i < MAXGPWSALARMS; i++) {
    GpwsAlarms[i].alarm.lastvalue = 0;
    GpwsAlarms[i].alarm.lasttriggertime = 0.0;
    GpwsAlarms[i].alarm.triggerscount = 0;
  }
  
  GpwsAlarms[0].alarm.triggervalue = 10;
  GpwsAlarms[0].sound= TEXT("GPWS0010.WAV");
  
  GpwsAlarms[1].alarm.triggervalue = 20;
  GpwsAlarms[1].sound= TEXT("GPWS0020.WAV");
  
  GpwsAlarms[2].alarm.triggervalue = 30;
  GpwsAlarms[2].sound= TEXT("GPWS0030.WAV");
  
  GpwsAlarms[3].alarm.triggervalue = 40;
  GpwsAlarms[3].sound= TEXT("GPWS0040.WAV");
  
  GpwsAlarms[4].alarm.triggervalue = 50;
  GpwsAlarms[4].sound= TEXT("GPWS0050.WAV");
  
  GpwsAlarms[5].alarm.triggervalue = 100;
  GpwsAlarms[5].sound= TEXT("GPWS0100.WAV");
  
  GpwsAlarms[6].alarm.triggervalue = 200;
  GpwsAlarms[6].sound= TEXT("GPWS0200.WAV");
  
  GpwsAlarms[7].alarm.triggervalue = 300;
  GpwsAlarms[7].sound= TEXT("GPWSMIN.WAV");
  
  GpwsAlarms[8].alarm.triggervalue = 400;
  GpwsAlarms[8].sound= TEXT("GPWS0400.WAV");
  
  GpwsAlarms[9].alarm.triggervalue = 500;
  GpwsAlarms[9].sound= TEXT("GPWS0500.WAV");
  
  GpwsAlarms[10].alarm.triggervalue = 1000;
  GpwsAlarms[10].sound= TEXT("GPWS1000.WAV");
  
  GpwsAlarms[11].alarm.triggervalue = 250;
  GpwsAlarms[11].sound= TEXT("GPWSTOOLOWT.WAV");
  
  GpwsAlarms[12].alarm.triggervalue = 350;
  GpwsAlarms[12].sound= TEXT("GPWSAPPMIN.WAV");

}

#if DEBUG_LKALARMS
#undef LKALARMSINTERVAL
#undef MAXLKALARMSTRIGGERS
#define LKALARMSINTERVAL 10
#define MAXLKALARMSTRIGGERS 3
#endif

// alarms in range 0-(MAXLKALARMS-1), that is  0-2
bool CheckAlarms(unsigned short al) {

  int i;

  // safe check
  if (al>=MAXLKALARMS) return false;

  // Alarms are working only with a valid GPS fix. No navigator, no alarms.
  if (GPS_INFO.NAVWarning) return false;

  // do we have a valid alarm request?
  if ( LKalarms[al].triggervalue == 0) return false;

  // We check for duplicates. We could do it only when config is changing, right now.
  // However, maybe we can have LK set automatically alarms in the future.
  // Duplicates filter is working giving priority to the lowest element in the list
  // We don't want more than 1 alarm for the same trigger value
  for (i=0; i<=al; i++) {
	if (i==al) continue; // do not check against ourselves
	// if a previous alarm has the same value, we are a duplicate
	if (LKalarms[al].triggervalue == LKalarms[i].triggervalue) {
		#if DEBUG_LKALARMS
		StartupStore(_T("...... Alarms: duplicate value [%d]=[%d] =<%d>\n"), al, i, LKalarms[i].triggervalue);
		#endif
		return false;
	}
  }

  // ok so this is not a duplicated alarm, lets check if we have overcounted
//  if (LKalarms[al].triggerscount >= MAXLKALARMSTRIGGERS) {
//	#if DEBUG_LKALARMS
//	StartupStore(_T("...... Alarms: count exceeded for [%d]\n"),al);
//	#endif
//	return false;
//  }

  // if too early we ignore it in any case
  if (GPS_INFO.Time < (LKalarms[al].lasttriggertime + LKALARMSINTERVAL)) {
	#if DEBUG_LKALARMS
	StartupStore(_T("...... Alarms: too early for [%d], still %.0f seconds to go\n"),al,
	(LKalarms[al].lasttriggertime + LKALARMSINTERVAL)- GPS_INFO.Time);
	#endif
	return false;
  }

  // So this is a potentially valid alarm to check

  //
  // First we check for altitude alarms , 0-2
  //
  if (al<3) {

	int agl=(int)CALCULATED_INFO.AltitudeAGL;

	// is this is the first valid sample?
	if (LKalarms[al].lastvalue==0) {
		LKalarms[al].lastvalue= agl;
		#if DEBUG_LKALARMS
		StartupStore(_T("...... Alarms: init lastvalue [%d] = %d\n"),al,LKalarms[al].lastvalue);
		#endif
		return false;
	}

	// if we were previously below trigger altitude
	if (LKalarms[al].lastvalue > LKalarms[al].triggervalue) {
		#if DEBUG_LKALARMS
		StartupStore(_T("...... Alarms: armed lastvalue [%d] = %d < trigger <%d>\n"),al,
		LKalarms[al].lastvalue,LKalarms[al].triggervalue);
		#endif
		// if we are now over the trigger altitude
		if (agl <= LKalarms[al].triggervalue) {
			#if DEBUG_LKALARMS
			StartupStore(_T("...... Alarms: RING [%d] = %d\n"),al,agl);
			#endif
			// bingo. first reset last value , update lasttime and counter
			LKalarms[al].lastvalue=0;
			LKalarms[al].triggerscount++;
			LKalarms[al].lasttriggertime = GPS_INFO.Time;
			return true;
		}
	}

	// otherwise simply update lastvalue
	LKalarms[al].lastvalue=agl;
	return false;

  } // end altitude alarms


  // other alarms here, or failed
  return false;

}

void CheckGpwsAlarms(void) {
    
    // Alarms are working only with a valid GPS fix. No navigator, no alarms.
    if (GPS_INFO.NAVWarning) {
        return;
    }
    
    int agl=(int)CALCULATED_INFO.AltitudeAGL;
    
    if (agl > 1000) {
        return; // optimization
    }
    
    for (int i = 0; i < MAXGPWSALARMS; i++) {

	// is this is the first valid sample?
	if (GpwsAlarms[i].alarm.lastvalue==0) {
		GpwsAlarms[i].alarm.lastvalue= agl;
		continue;
	}

	// if we were previously below trigger altitude
	if (GpwsAlarms[i].alarm.lastvalue > GpwsAlarms[i].alarm.triggervalue) {
		// if we are now over the trigger altitude
		if (agl <= GpwsAlarms[i].alarm.triggervalue) {
			// bingo. first reset last value , update lasttime and counter
			GpwsAlarms[i].alarm.lastvalue=0;
			GpwsAlarms[i].alarm.triggerscount++;
			GpwsAlarms[i].alarm.lasttriggertime = GPS_INFO.Time;
                        LKSound(GpwsAlarms[i].sound);
                        continue;
		}
	}

	// otherwise simply update lastvalue
	GpwsAlarms[i].alarm.lastvalue=agl;
    }
}


