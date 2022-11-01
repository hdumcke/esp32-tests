set(component_srcs "src/SCS.cpp"
                   "src/SCSCL.cpp"
                   "src/SCSerial.cpp"
                   "src/SMS_STS.cpp"
)

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES driver freertos
                       REQUIRES )
