#ifndef CRYPTOPP_CPU_H
#define CRYPTOPP_CPU_H

#include "Tcdefs.h"
#include "config.h"

	#define NEW_LINE
	#define INTEL_PREFIX ".intel_syntax prefix;"
	#define INTEL_NOPREFIX ".intel_syntax noprefix;"
	#define ATT_PREFIX ".att_syntax prefix;"
	#define ATT_NOPREFIX ".att_syntax noprefix;"




#define HasSSE2()	0
#define HasISSE()	0

#define HasMMX()	0
#define HasSSE42() 0
#define HasSSE41() 0
#define HasSAVX() 0
#define HasSAVX2() 0
#define HasSBMI2() 0
#define HasSSSE3() 0
#define HasAESNI() 0
#define HasCLMUL() 0
#define IsP4() 0
#define HasRDRAND() 0
#define HasRDSEED() 0
#define IsCpuIntel() 0
#define IsCpuAMD() 0
#define GetCacheLineSize()	CRYPTOPP_L1_CACHE_LINE_SIZE

#define DetectX86Features()
#define DisableCPUExtendedFeatures()

#endif
