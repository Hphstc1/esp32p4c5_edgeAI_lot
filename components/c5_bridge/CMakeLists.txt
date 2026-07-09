idf_component_register(
    SRCS "c5_sdio_probe.c" "c5_sdio_transport.c" "c5_at_sdio.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer freertos sdmmc
)
