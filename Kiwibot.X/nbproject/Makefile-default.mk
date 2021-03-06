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
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/OmniwheelBot.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/OmniwheelBot.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=globals.c main.c movement.c NU32.c sensing.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/globals.o ${OBJECTDIR}/main.o ${OBJECTDIR}/movement.o ${OBJECTDIR}/NU32.o ${OBJECTDIR}/sensing.o
POSSIBLE_DEPFILES=${OBJECTDIR}/globals.o.d ${OBJECTDIR}/main.o.d ${OBJECTDIR}/movement.o.d ${OBJECTDIR}/NU32.o.d ${OBJECTDIR}/sensing.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/globals.o ${OBJECTDIR}/main.o ${OBJECTDIR}/movement.o ${OBJECTDIR}/NU32.o ${OBJECTDIR}/sensing.o

# Source Files
SOURCEFILES=globals.c main.c movement.c NU32.c sensing.c


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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/OmniwheelBot.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=,--script="NU32bootloaded.ld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/globals.o: globals.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/globals.o.d 
	@${FIXDEPS} "${OBJECTDIR}/globals.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/globals.o.d" -o ${OBJECTDIR}/globals.o globals.c   
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c   
	
${OBJECTDIR}/movement.o: movement.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/movement.o.d 
	@${FIXDEPS} "${OBJECTDIR}/movement.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/movement.o.d" -o ${OBJECTDIR}/movement.o movement.c   
	
${OBJECTDIR}/NU32.o: NU32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/NU32.o.d 
	@${FIXDEPS} "${OBJECTDIR}/NU32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/NU32.o.d" -o ${OBJECTDIR}/NU32.o NU32.c   
	
${OBJECTDIR}/sensing.o: sensing.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/sensing.o.d 
	@${FIXDEPS} "${OBJECTDIR}/sensing.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/sensing.o.d" -o ${OBJECTDIR}/sensing.o sensing.c   
	
else
${OBJECTDIR}/globals.o: globals.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/globals.o.d 
	@${FIXDEPS} "${OBJECTDIR}/globals.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/globals.o.d" -o ${OBJECTDIR}/globals.o globals.c   
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c   
	
${OBJECTDIR}/movement.o: movement.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/movement.o.d 
	@${FIXDEPS} "${OBJECTDIR}/movement.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/movement.o.d" -o ${OBJECTDIR}/movement.o movement.c   
	
${OBJECTDIR}/NU32.o: NU32.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/NU32.o.d 
	@${FIXDEPS} "${OBJECTDIR}/NU32.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/NU32.o.d" -o ${OBJECTDIR}/NU32.o NU32.c   
	
${OBJECTDIR}/sensing.o: sensing.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/sensing.o.d 
	@${FIXDEPS} "${OBJECTDIR}/sensing.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/sensing.o.d" -o ${OBJECTDIR}/sensing.o sensing.c   
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/OmniwheelBot.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    NU32bootloaded.ld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_ICD3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/OmniwheelBot.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/OmniwheelBot.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   NU32bootloaded.ld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/OmniwheelBot.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	${MP_CC_DIR}/xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/OmniwheelBot.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
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

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
