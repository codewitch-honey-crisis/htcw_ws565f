cmake_minimum_required(VERSION 3.24)

# ESP-IDF component
idf_component_register(
    SRCS "ws565f.c"
    INCLUDE_DIRS "."
    REQUIRES driver freertos        # add this (or PRIV_REQUIRES — see below)
)
