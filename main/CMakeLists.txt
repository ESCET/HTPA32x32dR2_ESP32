idf_component_register(
    SRCS "main.c" "server.c" "connect.c" "htpa.c"
    INCLUDE_DIRS "."
)

spiffs_create_partition_image(spiffs  ../html FLASH_IN_PROJECT)