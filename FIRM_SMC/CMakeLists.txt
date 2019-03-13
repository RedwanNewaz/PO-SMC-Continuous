include(${QT_USE_FILE})
include_directories(${PROJECT_SOURCE_DIR}/FIRM_SMC/include)
set(HEADER_FILES include/StatisticalModelChecking.h include/TwoDPointRobotSMCSetup.h include/MCTS.h)
set(SOURCE_FILES  src/StatisticalModelChecking.cpp src/MCTS.cpp )
# Add the main as executable
add_executable (firmsmc-demo main_firmsmc.cpp ${HEADER_FILES} ${SOURCE_FILES})

# make sure libfcl.so is in /usr/local/lib/
target_link_libraries (firmsmc-demo
	bsp_lib
	${OMPLAPP_LIBRARY}
	${OMPLAPPBASE_LIBRARY}
	libfcl.so
)

