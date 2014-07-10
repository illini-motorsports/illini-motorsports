/******************************************************************************
 *
 *               Microchip Memory Disk Drive File System
 *
 ******************************************************************************
 * FileName:        FSconfig.h
 * Processor:       PIC18
 * Dependencies:    HardwareProfile.h and
 *					FSDefs.h
 * Compiler:        C18
 * Company:         Microchip Technology, Inc.
 * Version:         1.3.0
 *
 * Software License Agreement
 *
 * The software supplied herewith by Microchip Technology Incorporated
 * (the “Company”) for its PICmicro® Microcontroller is intended and
 * supplied to you, the Company’s customer, for use solely and
 * exclusively on Microchip PICmicro Microcontroller products. The
 * software is owned by the Company and/or its supplier, and is
 * protected under applicable copyright laws. All rights are reserved.
 * Any use in violation of the foregoing restrictions may subject the
 * user to criminal sanctions under applicable laws, as well as to
 * civil liability for the breach of the terms and conditions of this
 * license.
 *
 * THIS SOFTWARE IS PROVIDED IN AN “AS IS” CONDITION. NO WARRANTIES,
 * WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED
 * TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT,
 * IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL OR
 * CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 *
*******************************************************************************
	USER REVISON HISTORY
	note: modified to work with just PIC18F46K80
//
// 10/22/12 - removed other extra physical layers, confirmed location of data and FAT buffer,
//			- and removed the format function along with dynamic file allocation
//
// 10/24/12 - added MEDIA_SOFT_DETECT define
//
// 11/14/12 - set maximum number of files open for static allocation
//			- and turned off dynamic allocation along with directories
//

*******************************************************************************/

#include "HardwareProfile.h"
#include "FSDefs.h"

// Summary: A macro indicating the maximum number of concurrently open files
// Description: The FS_MAX_FILES_OPEN #define is only applicable when dynamic memory allocation is not used (FS_DYNAMIC_MEM is not defined).
//              This macro defines the maximum number of open files at any given time.  The amount of RAM used by FSFILE objects will
//              be equal to the size of an FSFILE object multipled by this macro value.  This value should be kept as small as possible
//              as dictated by the application.  This will reduce memory usage.
#define FS_MAX_FILES_OPEN 	1


// Summary: A macro defining the size of a sector
// Description: The MEDIA_SECTOR_SIZE macro will define the size of a sector on the FAT file system.  This value must equal 512 bytes,
//              1024 bytes, 2048 bytes, or 4096 bytes.  The value of a sector will usually be 512 bytes.
#define MEDIA_SECTOR_SIZE 		512



/* *******************************************************************************************************/
/************** Compiler options to enable/Disable Features based on user's application ******************/
/* *******************************************************************************************************/


// Summary: A macro to enable/disable file search functions.
// Description: The ALLOW_FILESEARCH definition can be commented out to disable file search functions in the library.  This will
//              prevent the use of the FindFirst and FindNext functions and reduce code size.
#define ALLOW_FILESEARCH
#define ALLOW_WRITES

// Summary: A macro to enable/disable FAT32 support.
// Description: The SUPPORT_FAT32 definition can be commented out to disable support for FAT32 functionality.  This will save a small amount
//              of code space.
#define SUPPORT_FAT32

// Summary: A macro to enable/disable LFN support.
// Description: The SUPPORT_FAT32 definition can be commented out to disable support for Long File Name functionality.
//
//#define SUPPORT_LFN


/**************************************************************************************************/
// Select a method for updating file timestamps
/**************************************************************************************************/

// Summary: A macro to enable RTCC based timestamp generation
// Description: The USEREALTIMECLOCK macro will configure the code to automatically
//              generate timestamp information for files from the RTCC module. The user
//              must enable and configure the RTCC module before creating or modifying
//              files.                                                                 
//#define USEREALTIMECLOCK

// Summary: A macro to enable manual timestamp generation
// Description: The USERDEFINEDCLOCK macro will allow the user to manually set
//              timestamp information using the SetClockVars function. The user will
//              need to set the time variables immediately before creating or closing a
//              file or directory.                                                    
#define USERDEFINEDCLOCK

// Summary: A macro to enable don't-care timestamp generation
/** NOTE: THIS WAY OF TRACKING TIME SCREWS WITH THE LIBRARY I.E. IT WON'T FUCKING WORK **/
// Description: The INCREMENTTIMESTAMP macro will set the create time of a file to a
//              static value and increment it when a file is updated. This timestamp
//              generation method should only be used in applications where file times
//              are not necessary.                                                    
//#define INCREMENTTIMESTAMP


#ifdef __18CXX
	#ifdef USEREALTIMECLOCK
		#error Not all PIC18 devices have a Real-time clock and calander module
	#endif
#endif

#ifndef USEREALTIMECLOCK
    #ifndef USERDEFINEDCLOCK
        #ifndef INCREMENTTIMESTAMP
            #error Please enable USEREALTIMECLOCK, USERDEFINEDCLOCK, or INCREMENTTIMESTAMP
        #endif
    #endif
#endif


/************************************************************************/
// Set this preprocessor option to '1' to use dynamic FSFILE object allocation.  It will
// be necessary to allocate a heap when dynamically allocating FSFILE objects.
// Set this option to '0' to use static FSFILE object allocation.
/************************************************************************/

#if 0
    // Summary: A macro indicating that FSFILE objects will be allocated dynamically
    // Description: The FS_DYNAMIC_MEM macro will cause FSFILE objects to be allocated from a dynamic heap.  If it is undefined,
    //              the file objects will be allocated using a static array.
	#define FS_DYNAMIC_MEM
	// Description: Function pointer to a dynamic memory allocation function
	#define FS_malloc	SRAMalloc
	// Description: Function pointer to a dynamic memory free function
	#define FS_free		SRAMfree
#endif

// Define the locations for the dataBuffer and FATbuffer
// PLEASE CHECK THE LINKER FILE for correct addresses
#define DATA_BUFFER_ADDRESS      0x300
#define FAT_BUFFER_ADDRESS       0x500


// Function definitions
// Associate the physical layer functions with the correct physical layer
//
// SD-SPI.c and .h library in use
//

// Description: Function pointer to the Media Initialize Physical Layer function
#define MDD_MediaInitialize     MDD_SDSPI_MediaInitialize

// Description: Function pointer to the Media Detect Physical Layer function
#define MDD_MediaDetect         MDD_SDSPI_MediaDetect

// Description: Function pointer to the Sector Read Physical Layer function
#define MDD_SectorRead          MDD_SDSPI_SectorRead

// Description: Function pointer to the Sector Write Physical Layer function
#define MDD_SectorWrite         MDD_SDSPI_SectorWrite

// Description: Function pointer to the Media Shutdown Physical Layer function
#define MDD_ShutdownMedia       MDD_SDSPI_ShutdownMedia

// Description: Function pointer to the Read Capacity Physical Layer function
#define MDD_ReadCapacity        MDD_SDSPI_ReadCapacity

// Description: Function pointer to the Read Sector Size Physical Layer Function
#define MDD_ReadSectorSize      MDD_SDSPI_ReadSectorSize
