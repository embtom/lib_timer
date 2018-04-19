project(lib_timer)

#However, the file(GLOB...) allows for wildcard additions:
file(GLOB SOURCES "*.c")


#add_definitions(-DTRACE)

#Generate the static library from the sources
add_library(${PROJECT_NAME} STATIC ${SOURCES})
target_link_libraries(${PROJECT_NAME} itf_lib_convention) 
target_include_directories(${PROJECT_NAME} PUBLIC ./)