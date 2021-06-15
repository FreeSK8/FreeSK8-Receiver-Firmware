# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

set(COMPONENT_SRCS "main.c"
    "lib/vesc/crc.c"
    "lib/vesc/buffer.c"
    "lib/vesc/datatypes.c"
    "lib/vesc/packet.c"
    )
set(COMPONENT_ADD_INCLUDEDIRS "./lib/vesc;")

register_component()

# Create a SPIFFS image from the contents of the 'font' directory
# that fits the partition named 'storage'. FLASH_IN_PROJECT indicates that
# the generated image should be flashed when the entire project is flashed to
# the target with 'idf.py -p PORT flash
spiffs_create_partition_image(storage ../filesystem FLASH_IN_PROJECT)