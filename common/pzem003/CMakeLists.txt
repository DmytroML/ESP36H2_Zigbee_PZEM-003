set(req driver freertos log esp_timer)

idf_component_register(
    SRCS "pzem003.c" 
    INCLUDE_DIRS "."
    REQUIRES  "${req}"
)

set_source_files_properties(pzem003.c
    PROPERTIES COMPILE_FLAGS
     -Wall -Wextra -Werror
)