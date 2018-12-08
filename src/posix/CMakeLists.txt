
lib_timer_add_architecture("posix")

if("${LIB_TIMER_ARCHITECTURE}" STREQUAL "posix")
    lib_timer_add_sourcefile_c(lib_timer_POSIX.c)
    lib_timer_add_private_definition(_GNU_SOURCE=1 __USE_GNU TRACE)
    lib_timer_add_dependencies(lib_thread)
    lib_timer_add_itf_header(include)
endif()



# project(lib_timer)

# #However, the file(GLOB...) allows for wildcard additions:
# file(GLOB SOURCES "*.c")


# #add_definitions(-DTRACE)

# #Generate the static library from the sources
# add_library(${PROJECT_NAME} STATIC ${SOURCES})
# target_compile_definitions(${PROJECT_NAME} PRIVATE _GNU_SOURCE=1 __USE_GNU TRACE)
# target_link_libraries(${PROJECT_NAME} lib_convention lib_thread)
# target_include_directories(${PROJECT_NAME} PUBLIC ./)
