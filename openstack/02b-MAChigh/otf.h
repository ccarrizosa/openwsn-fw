#ifndef __OTF_H
#define __OTF_H

/**
\addtogroup MAChigh
\{
\addtogroup otf
\{
*/

#include "opendefs.h"

//=========================== define ==========================================

//=========================== typedef =========================================

//=========================== module variables ================================

//=========================== prototypes ======================================

// admin
void      otf_init(void);
// notification from sixtop
void      otf_notif_addedCell(void);
void      otf_notif_removedCell(void);
// notification from schedule :(
// which component called this function related 
// to the scheduling algorithm, so it can 
// be other component if algorithm is different)
void      otf_notifyNewSlotframe(void);
/**
\}
\}
*/

#endif
