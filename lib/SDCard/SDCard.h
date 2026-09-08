#ifndef SD_CARD_H
#define SD_CARD_H

#include <cstddef>
#include "driver/gpio.h"
#include "sdmmc_cmd.h"

/**
 * @brief Minimal SD reader: mount a FAT filesystem and read whole files.
 *
 * This firmware only needs KeyStore::load_from_card() to work, so unlike
 * bally_OS's lib/SDCard (which also writes/streams/renames files and shares
 * mount ownership with a USB mass-storage host) this class does not include
 * TinyShell/Logger/Format at all -- see the plan's note on the SDCard<->
 * Logger<->RobotSettings<->OTAUpdater include cycle that forced bally_OS's
 * host build into lib_ldf_mode=chain+. Keeping this class free of that cycle
 * means no such workaround is needed here.
 *
 * Uses the SDMMC (not SDSPI) peripheral in 1-bit mode: the AI-Thinker
 * ESP32-CAM's SD slot is hardwired to the HS2 pins (CLK=GPIO14, CMD=GPIO15,
 * D0=GPIO2), the same slot the board's stock camera+SD example code uses.
 * 1-bit mode only needs those three signals, leaving D1(GPIO4)/D2(GPIO12)/
 * D3(GPIO13) free -- and none of CLK/CMD/D0 collide with the camera pinout
 * already in src/Camera.cpp (0,5,18,19,21,22,23,25,26,27,32,34,35,36,39).
 *
 * Same public shape (begin/is_mounted/read_file) bally_OS's KeyStore.cpp
 * already expects under its #if defined(ESP_PLATFORM) branch, copied
 * verbatim from bally_OS/lib/KeyStore.
 */
class SDCard {
public:
    explicit SDCard(const char* mount_point = "/sdcard");
    ~SDCard();

    SDCard(const SDCard&) = delete;
    SDCard& operator=(const SDCard&) = delete;

    /**
     * @brief Initialize the SDMMC peripheral and mount FAT.
     * @return true when the card is mounted and ready for read_file().
     */
    bool begin();

    /** @brief Unmount the filesystem. */
    void end();

    bool is_mounted() const { return mounted_; }
    const char* mount_point() const { return mount_point_; }

    /**
     * @brief Read up to capacity bytes from the beginning of a file.
     *
     * @param path Relative path or complete path below the mount point.
     * @param buffer Destination buffer.
     * @param capacity Maximum number of bytes to read.
     * @param bytes_read Optional output with the number of bytes read.
     * @return true when the read completes without a file error.
     */
    bool read_file(const char* path, void* buffer, size_t capacity,
                   size_t* bytes_read = nullptr) const;

    /**
     * @brief Write a whole file, replacing any previous content.
     *
     * @param path Relative path or complete path below the mount point.
     * @param data Bytes to write. May be null only when length is 0.
     * @param length Number of bytes to write.
     * @return true when the file was created/replaced, written, flushed and
     * closed without error.
     */
    bool write_file(const char* path, const void* data, size_t length) const;

private:
    static constexpr size_t kMaxPathLength = 256;

    bool make_path(const char* path, char* full_path, size_t capacity) const;

    const char* mount_point_;
    sdmmc_card_t* card_ = nullptr;
    bool mounted_ = false;
};

#endif  // SD_CARD_H
