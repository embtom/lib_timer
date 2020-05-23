
lib_timer_add_architecture("posix")

if("${LIB_TIMER_ARCHITECTURE}" STREQUAL "posix")
    lib_timer_add_sourcefile_c(lib_timer_POSIX.c)
    lib_timer_add_private_definition(_GNU_SOURCE=1 __USE_GNU TRACE)
    lib_timer_add_dependencies(lib_thread)
    lib_timer_add_itf_header(include)
endif()
