# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

set(srcs  ../../wifibroadcast/src/fec.c )

idf_component_register(SRCS  "esp_wb.cpp" "main.cpp" "${srcs}"
                    INCLUDE_DIRS "." "../../wifibroadcast/src" )