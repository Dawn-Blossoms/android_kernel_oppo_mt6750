#ifndef _LC898214AF_H
#define _LC898214AF_H

#include <linux/ioctl.h>
/* #include "kd_imgsensor.h" */

#define LC898214AF_MAGIC 'A'
/* IOCTRL(inode * ,file * ,cmd ,arg ) */


/* Structures */
typedef struct {
/* current position */
	u32 u4CurrentPosition;
/* macro position */
	u32 u4MacroPosition;
/* Infiniti position */
	u32 u4InfPosition;
/* Motor Status */
	bool bIsMotorMoving;
/* Motor Open? */
	bool bIsMotorOpen;
/* Support SR? */
	bool bIsSupportSR;
} stLC898214AF_MotorInfo;

/* Control commnad */
/* S means "set through a ptr" */
/* T means "tell by a arg value" */
/* G means "get by a ptr" */
/* Q means "get by return a value" */
/* X means "switch G and S atomically" */
/* H means "switch T and Q atomically" */
#define LC898214AFIOC_G_MOTORINFO _IOR(LC898214AF_MAGIC, 0, stLC898214AF_MotorInfo)

#define LC898214AFIOC_T_MOVETO _IOW(LC898214AF_MAGIC, 1, u32)

#define LC898214AFIOC_T_SETINFPOS _IOW(LC898214AF_MAGIC, 2, u32)

#define LC898214AFIOC_T_SETMACROPOS _IOW(LC898214AF_MAGIC, 3, u32)

#else
#endif
