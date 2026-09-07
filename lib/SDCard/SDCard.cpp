#include <SDCard.h>

#include "driver/sdmmc_host.h"
#include "esp_vfs_fat.h"

#include <cstdio>
#include <cstring>

namespace {

// AI-Thinker ESP32-CAM's hardwired SD slot (HS2), 1-bit mode.
constexpr gpio_num_t kPinClk = GPIO_NUM_14;
constexpr gpio_num_t kPinCmd = GPIO_NUM_15;
constexpr gpio_num_t kPinD0 = GPIO_NUM_2;

}  // namespace

SDCard::SDCard(const char* mount_point) : mount_point_(mount_point) {}

SDCard::~SDCard() { end(); }

bool SDCard::begin() {
    if (mounted_) return true;
    if (mount_point_ == nullptr || mount_point_[0] != '/') return false;

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.flags = SDMMC_HOST_FLAG_1BIT;
    host.slot = SDMMC_HOST_SLOT_1;
    host.max_freq_khz = SDMMC_FREQ_PROBING;

    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.clk = kPinClk;
    slot_config.cmd = kPinCmd;
    slot_config.d0 = kPinD0;
    slot_config.width = 1;

    // Never reformat a card that fails to mount: it may hold bally.key and
    // nothing else on it. A robot that cannot read the key stays fail-closed
    // (RadioSeal never authenticates), which is the correct failure mode.
    esp_vfs_fat_mount_config_t mount_config{};
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 2;
    mount_config.allocation_unit_size = 16U * 1024U;

    sdmmc_card_t* card = nullptr;
    if (esp_vfs_fat_sdmmc_mount(mount_point_, &host, &slot_config,
                                &mount_config, &card) != ESP_OK) {
        return false;
    }

    card_ = card;
    mounted_ = true;
    return true;
}

void SDCard::end() {
    if (mounted_) {
        esp_vfs_fat_sdcard_unmount(mount_point_, card_);
        mounted_ = false;
        card_ = nullptr;
    }
}

bool SDCard::make_path(const char* path, char* full_path,
                       size_t capacity) const {
    if (!mounted_ || path == nullptr || path[0] == '\0' ||
        full_path == nullptr || capacity == 0) {
        return false;
    }

    const size_t mount_length = strlen(mount_point_);
    int written = 0;

    // Accept "/sdcard/bally.key", "/bally.key" and "bally.key".
    if (strncmp(path, mount_point_, mount_length) == 0 &&
        (path[mount_length] == '/' || path[mount_length] == '\0')) {
        written = snprintf(full_path, capacity, "%s", path);
    } else if (path[0] == '/') {
        written = snprintf(full_path, capacity, "%s%s", mount_point_, path);
    } else {
        written = snprintf(full_path, capacity, "%s/%s", mount_point_, path);
    }

    return written > 0 && static_cast<size_t>(written) < capacity;
}

bool SDCard::read_file(const char* path, void* buffer, size_t capacity,
                       size_t* bytes_read) const {
    if (buffer == nullptr) return false;

    char full_path[kMaxPathLength];
    if (!make_path(path, full_path, sizeof(full_path))) return false;

    FILE* file = fopen(full_path, "rb");
    if (file == nullptr) return false;

    const size_t read = fread(buffer, 1, capacity, file);
    const bool had_error = ferror(file) != 0;
    fclose(file);
    if (had_error) return false;

    if (bytes_read != nullptr) *bytes_read = read;
    return true;
}
