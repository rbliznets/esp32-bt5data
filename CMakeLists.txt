idf_component_register(SRCS "CBTTask.cpp" "CMacStore.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES task bt nvs_flash nlohmann-json dataformat)
target_compile_options(${COMPONENT_LIB} PRIVATE -Os)
