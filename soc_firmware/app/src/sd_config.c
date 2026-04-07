/*
 * Copyright (c) 2025
 * SPDX-License-Identifier: Apache-2.0
 *
 * SD Card configuration storage implementation.
 *
 * JSON serialization / parsing is handled by config_json.c;
 * this module only manages the SD card I/O.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/storage/disk_access.h>
#include <zephyr/fs/fs.h>
#include <zephyr/logging/log.h>
#include <ff.h>
#include <string.h>

#include "sd_config.h"
#include "config_json.h"

LOG_MODULE_REGISTER(sd_config, LOG_LEVEL_INF);

/* ---- Filesystem state ---- */
static FATFS fat_fs;
static bool sd_mounted;
static bool config_dirty;
static struct k_mutex sd_mutex;
static enum sd_config_load_status load_status = SD_CONFIG_NOT_LOADED;

static struct fs_mount_t sd_mount = {
.type = FS_FATFS,
.fs_data = &fat_fs,
.mnt_point = SD_MOUNT_POINT,
};

/* ---- JSON buffer for serialization ---- */
#define JSON_BUF_SIZE  8192
static char json_buf[JSON_BUF_SIZE];

/* ================================================================
 * Public API: Initialize SD card
 * ================================================================ */
int sd_config_init(void)
{
int ret;

k_mutex_init(&sd_mutex);

/* Check if disk is ready */
ret = disk_access_init("SD");
if (ret != 0) {
LOG_WRN("SD card not detected (ret=%d)", ret);
return -ENODEV;
}

/* Mount FAT filesystem */
ret = fs_mount(&sd_mount);
if (ret != 0) {
LOG_ERR("Failed to mount SD card: %d", ret);
return ret;
}

sd_mounted = true;
LOG_INF("SD card mounted at %s", SD_MOUNT_POINT);
return 0;
}

/* ================================================================
 * Public API: Check if SD is ready
 * ================================================================ */
bool sd_config_is_ready(void)
{
return sd_mounted;
}

/* ================================================================
 * Public API: Load configuration from SD card
 * ================================================================ */
int sd_config_load(void)
{
struct fs_file_t file;
struct fs_dirent entry;
int ret;

if (!sd_mounted) {
LOG_WRN("SD card not mounted, cannot load config");
load_status = SD_CONFIG_LOAD_NO_CARD;
return -ENODEV;
}

k_mutex_lock(&sd_mutex, K_FOREVER);

/* Check if config file exists */
ret = fs_stat(SD_CONFIG_FILE_PATH, &entry);
if (ret != 0) {
LOG_INF("No config file found at %s, using defaults",
SD_CONFIG_FILE_PATH);
load_status = SD_CONFIG_LOAD_NO_FILE;
k_mutex_unlock(&sd_mutex);
return -ENOENT;
}

if (entry.size >= JSON_BUF_SIZE) {
LOG_ERR("Config file too large: %zu bytes", entry.size);
load_status = SD_CONFIG_LOAD_ERROR;
k_mutex_unlock(&sd_mutex);
return -ENOMEM;
}

/* Open and read file */
fs_file_t_init(&file);
ret = fs_open(&file, SD_CONFIG_FILE_PATH, FS_O_READ);
if (ret != 0) {
LOG_ERR("Failed to open config file: %d", ret);
load_status = SD_CONFIG_LOAD_ERROR;
k_mutex_unlock(&sd_mutex);
return ret;
}

ssize_t bytes = fs_read(&file, json_buf, entry.size);
fs_close(&file);

if (bytes < 0) {
LOG_ERR("Failed to read config file: %zd", bytes);
load_status = SD_CONFIG_LOAD_ERROR;
k_mutex_unlock(&sd_mutex);
return (int)bytes;
}

json_buf[bytes] = '\0';
LOG_INF("Read %zd bytes from config file", bytes);

/* Parse and apply configuration */
config_json_parse_and_apply(json_buf);

load_status = SD_CONFIG_LOAD_OK;
k_mutex_unlock(&sd_mutex);
return 0;
}

/* ================================================================
 * Public API: Save configuration to SD card
 * ================================================================ */
int sd_config_save(void)
{
struct fs_file_t file;
int ret;

if (!sd_mounted) {
LOG_WRN("SD card not mounted, cannot save config");
return -ENODEV;
}

k_mutex_lock(&sd_mutex, K_FOREVER);

/* Serialize current config to JSON */
int len = config_json_serialize(json_buf, JSON_BUF_SIZE);
if (len < 0 || len >= JSON_BUF_SIZE) {
LOG_ERR("Failed to serialize config");
k_mutex_unlock(&sd_mutex);
return -ENOMEM;
}

/* Open file for writing (truncate if exists) */
fs_file_t_init(&file);
ret = fs_open(&file, SD_CONFIG_FILE_PATH,
      FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
if (ret != 0) {
LOG_ERR("Failed to open config file for writing: %d", ret);
k_mutex_unlock(&sd_mutex);
return ret;
}

/* Write JSON data */
ssize_t written = fs_write(&file, json_buf, len);
ret = fs_close(&file);

if (written != len) {
LOG_ERR("Failed to write config file: %zd", written);
k_mutex_unlock(&sd_mutex);
return -EIO;
}

config_dirty = false;
LOG_INF("Configuration saved to SD card (%d bytes)", len);

k_mutex_unlock(&sd_mutex);
return 0;
}

/* ================================================================
 * Public API: Mark config as dirty
 * ================================================================ */
void sd_config_mark_dirty(void)
{
config_dirty = true;
}

/* ================================================================
 * Public API: Flush pending changes
 * ================================================================ */
int sd_config_flush(void)
{
if (!config_dirty) {
return 0;
}
return sd_config_save();
}

/* ================================================================
 * Public API: Format SD card
 * ================================================================ */
int sd_config_format(void)
{
int ret;

if (!sd_mounted) {
LOG_WRN("SD card not mounted, cannot format");
return -ENODEV;
}

k_mutex_lock(&sd_mutex, K_FOREVER);

LOG_WRN("Formatting SD card...");

/* Unmount first */
ret = fs_unmount(&sd_mount);
if (ret != 0 && ret != -EINVAL) {
LOG_ERR("Failed to unmount SD card: %d", ret);
k_mutex_unlock(&sd_mutex);
return ret;
}

/* Format using FatFS f_mkfs with MKFS_PARM structure */
MKFS_PARM mkfs_opt = {
.fmt = FM_FAT32,
.n_fat = 0,    /* Use default */
.align = 0,    /* Use default */
.n_root = 0,   /* Use default */
.au_size = 0   /* Use default cluster size */
};
FRESULT fret = f_mkfs("SD:", &mkfs_opt, json_buf, JSON_BUF_SIZE);
if (fret != FR_OK) {
LOG_ERR("Failed to format SD card: %d", fret);
/* Try to remount */
fs_mount(&sd_mount);
k_mutex_unlock(&sd_mutex);
return -EIO;
}

/* Remount */
ret = fs_mount(&sd_mount);
if (ret != 0) {
LOG_ERR("Failed to remount SD card after format: %d", ret);
sd_mounted = false;
k_mutex_unlock(&sd_mutex);
return ret;
}

load_status = SD_CONFIG_LOAD_NO_FILE;
LOG_INF("SD card formatted successfully");

k_mutex_unlock(&sd_mutex);
return 0;
}

/* ================================================================
 * Public API: Get load status
 * ================================================================ */
enum sd_config_load_status sd_config_get_load_status(void)
{
return load_status;
}

/* ================================================================
 * Public API: Get status string
 * ================================================================ */
const char *sd_config_status_str(enum sd_config_load_status status)
{
switch (status) {
case SD_CONFIG_NOT_LOADED:
return "not_loaded";
case SD_CONFIG_LOAD_OK:
return "ok";
case SD_CONFIG_LOAD_NO_FILE:
return "no_file";
case SD_CONFIG_LOAD_NO_CARD:
return "no_card";
case SD_CONFIG_LOAD_ERROR:
return "error";
default:
return "unknown";
}
}
