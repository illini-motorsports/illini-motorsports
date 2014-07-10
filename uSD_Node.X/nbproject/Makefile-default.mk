#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=cof
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/uSD_Node.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/uSD_Node.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/DAQ.o ${OBJECTDIR}/FSIO.o ${OBJECTDIR}/SD-SPI.o ${OBJECTDIR}/_ext/2123637668/ECAN.o
POSSIBLE_DEPFILES=${OBJECTDIR}/DAQ.o.d ${OBJECTDIR}/FSIO.o.d ${OBJECTDIR}/SD-SPI.o.d ${OBJECTDIR}/_ext/2123637668/ECAN.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/DAQ.o ${OBJECTDIR}/FSIO.o ${OBJECTDIR}/SD-SPI.o ${OBJECTDIR}/_ext/2123637668/ECAN.o


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/uSD_Node.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=18F46K80
MP_PROCESSOR_OPTION_LD=18f46k80
MP_LINKER_DEBUG_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/DAQ.o: DAQ.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/DAQ.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DLOGGING -I"../ECAN.X" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/DAQ.o   DAQ.c 
	@${DEP_GEN} -d ${OBJECTDIR}/DAQ.o 
	@${FIXDEPS} "${OBJECTDIR}/DAQ.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/FSIO.o: FSIO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/FSIO.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DLOGGING -I"../ECAN.X" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/FSIO.o   FSIO.c 
	@${DEP_GEN} -d ${OBJECTDIR}/FSIO.o 
	@${FIXDEPS} "${OBJECTDIR}/FSIO.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/SD-SPI.o: SD-SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/SD-SPI.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DLOGGING -I"../ECAN.X" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/SD-SPI.o   SD-SPI.c 
	@${DEP_GEN} -d ${OBJECTDIR}/SD-SPI.o 
	@${FIXDEPS} "${OBJECTDIR}/SD-SPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/2123637668/ECAN.o: ../ECAN.X/ECAN.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/2123637668 
	@${RM} ${OBJECTDIR}/_ext/2123637668/ECAN.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -p$(MP_PROCESSOR_OPTION) -DLOGGING -I"../ECAN.X" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/2123637668/ECAN.o   ../ECAN.X/ECAN.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/2123637668/ECAN.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2123637668/ECAN.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
else
${OBJECTDIR}/DAQ.o: DAQ.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/DAQ.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DLOGGING -I"../ECAN.X" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/DAQ.o   DAQ.c 
	@${DEP_GEN} -d ${OBJECTDIR}/DAQ.o 
	@${FIXDEPS} "${OBJECTDIR}/DAQ.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/FSIO.o: FSIO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/FSIO.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DLOGGING -I"../ECAN.X" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/FSIO.o   FSIO.c 
	@${DEP_GEN} -d ${OBJECTDIR}/FSIO.o 
	@${FIXDEPS} "${OBJECTDIR}/FSIO.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/SD-SPI.o: SD-SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/SD-SPI.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DLOGGING -I"../ECAN.X" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/SD-SPI.o   SD-SPI.c 
	@${DEP_GEN} -d ${OBJECTDIR}/SD-SPI.o 
	@${FIXDEPS} "${OBJECTDIR}/SD-SPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
${OBJECTDIR}/_ext/2123637668/ECAN.o: ../ECAN.X/ECAN.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/2123637668 
	@${RM} ${OBJECTDIR}/_ext/2123637668/ECAN.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE) -p$(MP_PROCESSOR_OPTION) -DLOGGING -I"../ECAN.X" -ms -oa- -Ls  -I ${MP_CC_DIR}\\..\\h  -fo ${OBJECTDIR}/_ext/2123637668/ECAN.o   ../ECAN.X/ECAN.c 
	@${DEP_GEN} -d ${OBJECTDIR}/_ext/2123637668/ECAN.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/2123637668/ECAN.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c18 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/uSD_Node.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    18f46k80_SD_SPI.lkr
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) "18f46k80_SD_SPI.lkr"  -p$(MP_PROCESSOR_OPTION_LD)  -w -x -u_DEBUG   -z__MPLAB_BUILD=1  -u_CRUNTIME -z__MPLAB_DEBUG=1 -z__MPLAB_DEBUGGER_PK3=1 $(MP_LINKER_DEBUG_OPTION) -l ${MP_CC_DIR}\\..\\lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/uSD_Node.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}   
else
dist/${CND_CONF}/${IMAGE_TYPE}/uSD_Node.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   18f46k80_SD_SPI.lkr
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_LD} $(MP_EXTRA_LD_PRE) "18f46k80_SD_SPI.lkr"  -p$(MP_PROCESSOR_OPTION_LD)  -w    -z__MPLAB_BUILD=1  -u_CRUNTIME -l ${MP_CC_DIR}\\..\\lib  -o dist/${CND_CONF}/${IMAGE_TYPE}/uSD_Node.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}   
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
