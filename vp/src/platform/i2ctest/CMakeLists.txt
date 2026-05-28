file(GLOB_RECURSE HEADERS ${CMAKE_CURRENT_SOURCE_DIR}/*.h)
set(SOURCES
	main.cpp
	${HEADERS})
set(LIBS platform-common gdb-mc
	${Boost_LIBRARIES}
	systemc
	pthread)

add_executable(i2ctest-vp ${SOURCES})
target_compile_definitions(i2ctest-vp PUBLIC
	TARGET_RV32
	NUM_CORES=1)
target_link_libraries(i2ctest-vp rv32 ${LIBS})

INSTALL(TARGETS i2ctest-vp RUNTIME DESTINATION bin)
