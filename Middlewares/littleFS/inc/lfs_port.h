#ifndef LFS_PORT_H
#define LFS_PORT_H

#include "lfs.h"
#include <stdio.h>
#include <string.h>
#include <sfud.h>

// W25Q64 ���ò���
#define LFS_BLOCK_SIZE 4096    // 4KB ������С
#define LFS_BLOCK_COUNT 2048   // 8MB / 4KB = 2048������
#define LFS_PROG_SIZE 256      // ҳ��̴�С
#define LFS_READ_SIZE 256      // ��ȡ��С
#define LFS_LOOKAHEAD_SIZE 128 // ���п���һ�������С
#define LFS_CACHE_SIZE 256     // �����С

// LittleFS�ӿں�������
int lfs_flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
int lfs_flash_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
int lfs_flash_erase(const struct lfs_config *c, lfs_block_t block);
int lfs_flash_sync(const struct lfs_config *c);

// ȫ�����ýṹ
extern struct lfs_config lfs_cfg;
extern lfs_t lfs_fs;

// ����������
extern uint8_t lfs_read_buffer[LFS_CACHE_SIZE];
extern uint8_t lfs_prog_buffer[LFS_CACHE_SIZE];
extern uint8_t lfs_lookahead_buffer[LFS_LOOKAHEAD_SIZE];

// LittleFS��ʼ���͹��غ���
int lfs_port_init(void);
int lfs_port_mount(void);
int lfs_port_format(void);

void littlefs_demo(void);
void initialize_and_test_LittleFS(void);

#endif /* LFS_PORT_H */
