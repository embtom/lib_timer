project (lib_timer)

cmake_minimum_required(VERSION 2.6)
add_subdirectory(src)

######################################################################################
# Build configurator
######################################################################################
option(LIB_TIMER_EXTERNAL_STREAMROUTER "Usage of the system iotty library for logging" ${LIB_TIMER_EXTERNAL_STREAMROUTER})

# Selector to choose architecture to build lib_timer
set(LIB_TIMER_ARCHITECTURE ${LIB_TIMER_ARCHITECTURE} CACHE STRING "Architecture to build lib_timer on")
get_property(architectures GLOBAL PROPERTY LIB_TIMER_ARCHITECTURES)
list(SORT architectures)
set_property(CACHE LIB_TIMER_ARCHITECTURE PROPERTY STRINGS None ${architectures})

#######################################################################################
# Check LIB_TIMER_ARCHITECTURE has a valid value
#######################################################################################
if("${LIB_TIMER_ARCHITECTURE}" STREQUAL "None")
message(FATAL_ERROR "No valid architecture chosen! Please check LIB_TIMER_ARCHITECTURE.")
endif("${LIB_TIMER_ARCHITECTURE}" STREQUAL "None")

#######################################################################################
# Get Architecture specific build paramters
#######################################################################################
GET_PROPERTY(LIB_TIMER_SOURCE_C            GLOBAL PROPERTY LIB_TIMER_ARCH_SOURCE_C)
GET_PROPERTY(LIB_TIMER_PRIVATE_DEFINITION  GLOBAL PROPERTY LIB_TIMER_ARCH_PRIVATE_DEFINITION)
GET_PROPERTY(LIB_TIMER_PUBLIC_DEFINITION   GLOBAL PROPERTY LIB_TIMER_ARCH_PUBLIC_DEFINITION)
GET_PROPERTY(LIB_TIMER_DEPEND              GLOBAL PROPERTY LIB_TIMER_ARCH_DEPEND)
GET_PROPERTY(LIB_TIMER_ITF_HEADER          GLOBAL PROPERTY LIB_TIMER_ARCH_ITF_HEADER)

if (LIB_TIMER_EXTERNAL_STREAMROUTER)
    find_package(streamRouter REQUIRED)
    SET(LIB_TIMER_DEPEND ${LIB_TIMER_DEPEND} lib_convention EMBTOM::streamRouter)
else()
    SET(LIB_TIMER_DEPEND ${LIB_TIMER_DEPEND} lib_convention lib_log)
endif()
SET(LIB_TIMER_PRIVATE_DEFINITION ${LIB_TIMER_PRIVATE_DEFINITION} TRACE )

message(STATUS "LIB_TIMER : selected architecture ${LIB_TIMER_ARCHITECTURE}")

#######################################################################################
#Build target settings
#######################################################################################
add_library(${PROJECT_NAME} STATIC ${LIB_TIMER_SOURCE_C})
target_link_libraries(${PROJECT_NAME} ${LIB_TIMER_DEPEND})
target_include_directories(${PROJECT_NAME} PUBLIC ./include ${LIB_TIMER_ITF_HEADER})
target_compile_definitions(${PROJECT_NAME} PRIVATE ${LIB_TIMER_PRIVATE_DEFINITION})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${LIB_TIMER_PUBLIC_DEFINITION})
