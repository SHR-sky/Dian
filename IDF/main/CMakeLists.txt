idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS ".")

set(COMPONENT_REQUIRES
    LED
    myusart
    myiic
    WiFi
    )
