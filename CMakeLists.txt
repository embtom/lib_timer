project(lib_timer)

#However, the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "*.c")


#add_definitions(-DTRACE)

#Generate the static library from the sources
add_library(${PROJECT_NAME} STATIC ${SOURCES})
#target_compile_definitions(${PROJECT_NAME} PRIVATE _GNU_SOURCE=1 __USE_GNU TRACE)
target_link_libraries(${PROJECT_NAME} itf_lib_convention lib_thread dev_framework_STM32F1_CUBE lib_isr)
target_include_directories(${PROJECT_NAME} PUBLIC ./)
