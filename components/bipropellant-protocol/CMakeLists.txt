set(component_srcs "cobsr.c" "machine_protocol.c" "protocol.c" "ascii_protocol.c"
)

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "."
                       PRIV_INCLUDE_DIRS ""
                       REQUIRES )
