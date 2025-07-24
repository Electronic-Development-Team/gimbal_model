#include "lfs_port.h"
#include "sfud.h"
#include <string.h>

// SFUD������
#define SFUD_DEMO_TEST_BUFFER_SIZE 1024
static uint8_t sfud_demo_test_buf[SFUD_DEMO_TEST_BUFFER_SIZE];

// LittleFS����������
uint8_t lfs_read_buffer[LFS_CACHE_SIZE];
uint8_t lfs_prog_buffer[LFS_CACHE_SIZE];
uint8_t lfs_lookahead_buffer[LFS_LOOKAHEAD_SIZE];

// LittleFS�ļ�ϵͳʵ��
lfs_t lfs_fs;

// SFUD Flash�豸ָ��
static const sfud_flash *flash = NULL;
/**
 * @brief ����ʧ��ʱ�Ļص�����
 *
 * @param expr ʧ�ܵı��ʽ
 * @param file ʧ�ܵ��ļ�
 * @param line ʧ�ܵ��к�
 */
void __aeabi_assert(const char *expr, const char *file, int line)
{
  // �򵥵�assertʵ�֣����Ը�����Ҫ�޸�
  (void)expr;
  (void)file;
  (void)line;
  // �ڵ���ģʽ�¿�����Ӵ�ӡ��ϵ�
  while (1)
    ; // ��������ѭ�������߿�������ϵͳ
}

/**
 * @brief LittleFS��ȡ����
 * @param c LittleFS���ýṹָ��
 * @param block ���
 * @param off ����ƫ��
 * @param buffer ��ȡ���ݻ�����
 * @param size ��ȡ���ݴ�С
 * @return 0�ɹ�����ֵʧ��
 */
int lfs_flash_read(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
  if (flash == NULL)
  {
    flash = sfud_get_device(SFUD_W25_DEVICE_INDEX);
    if (flash == NULL)
    {
      return LFS_ERR_IO;
    }
  }

  // ����ʵ�ʵ�ַ����� * ���С + ����ƫ��
  uint32_t addr = block * c->block_size + off;

  // ʹ��SFUD��ȡ����
  sfud_err result = sfud_read(flash, addr, size, (uint8_t *)buffer);

  return (result == SFUD_SUCCESS) ? LFS_ERR_OK : LFS_ERR_IO;
}

/**
 * @brief LittleFS��̣�д�룩����
 * @param c LittleFS���ýṹָ��
 * @param block ���
 * @param off ����ƫ��
 * @param buffer д�����ݻ�����
 * @param size д�����ݴ�С
 * @return 0�ɹ�����ֵʧ��
 */
int lfs_flash_prog(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
  if (flash == NULL)
  {
    flash = sfud_get_device(SFUD_W25_DEVICE_INDEX);
    if (flash == NULL)
    {
      return LFS_ERR_IO;
    }
  }

  // ����ʵ�ʵ�ַ����� * ���С + ����ƫ��
  uint32_t addr = block * c->block_size + off;

  // ʹ��SFUDд������
  sfud_err result = sfud_write(flash, addr, size, (const uint8_t *)buffer);

  return (result == SFUD_SUCCESS) ? LFS_ERR_OK : LFS_ERR_IO;
}

/**
 * @brief LittleFS��������
 * @param c LittleFS���ýṹָ��
 * @param block Ҫ�����Ŀ��
 * @return 0�ɹ�����ֵʧ��
 */
int lfs_flash_erase(const struct lfs_config *c, lfs_block_t block)
{
  if (flash == NULL)
  {
    flash = sfud_get_device(SFUD_W25_DEVICE_INDEX);
    if (flash == NULL)
    {
      return LFS_ERR_IO;
    }
  }

  // ����ʵ�ʵ�ַ����� * ���С
  uint32_t addr = block * c->block_size;

  // ʹ��SFUD����������4KB��
  sfud_err result = sfud_erase(flash, addr, c->block_size);

  return (result == SFUD_SUCCESS) ? LFS_ERR_OK : LFS_ERR_IO;
}

/**
 * @brief LittleFSͬ������
 * @param c LittleFS���ýṹָ��
 * @return 0�ɹ�����ֵʧ��
 */
int lfs_flash_sync(const struct lfs_config *c)
{
  // Flash������ͬ���ģ�����Ҫ�������
  return LFS_ERR_OK;
}

// LittleFS���ýṹ
struct lfs_config lfs_cfg = {
    // ���豸��������
    .read = lfs_flash_read,
    .prog = lfs_flash_prog,
    .erase = lfs_flash_erase,
    .sync = lfs_flash_sync,

    // ���豸����
    .read_size = LFS_READ_SIZE,
    .prog_size = LFS_PROG_SIZE,
    .block_size = LFS_BLOCK_SIZE,
    .block_count = LFS_BLOCK_COUNT,
    .cache_size = LFS_CACHE_SIZE,
    .lookahead_size = LFS_LOOKAHEAD_SIZE,
    .block_cycles = 500,

    // ��ѡ������
    .read_buffer = lfs_read_buffer,
    .prog_buffer = lfs_prog_buffer,
    .lookahead_buffer = lfs_lookahead_buffer,
};

/**
 * @brief ��ʼ��LittleFS��ֲ��
 * @return 0�ɹ�����ֵʧ��
 */
int lfs_port_init(void)
{
  // ��ʼ��SFUD
  if (sfud_init() != SFUD_SUCCESS)
  {
    return -1;
  }

  // ��ȡFlash�豸
  flash = sfud_get_device(SFUD_W25_DEVICE_INDEX);
  if (flash == NULL)
  {
    return -1;
  }

  return 0;
}

/**
 * @brief ����LittleFS�ļ�ϵͳ
 * @return 0�ɹ�����ֵʧ��
 */
int lfs_port_mount(void)
{
  // �����ļ�ϵͳ
  int err = lfs_mount(&lfs_fs, &lfs_cfg);

  // �������ʧ�ܣ�������Ҫ��ʽ��
  if (err)
  {
    // ��ʽ���ļ�ϵͳ
    lfs_format(&lfs_fs, &lfs_cfg);

    // ���¹���
    err = lfs_mount(&lfs_fs, &lfs_cfg);
  }

  return err;
}

/**
 * @brief ��ʽ��LittleFS�ļ�ϵͳ
 * @return 0�ɹ�����ֵʧ��
 */
int lfs_port_format(void)
{
  return lfs_format(&lfs_fs, &lfs_cfg);
}

/**
 * @brief LittleFS��ʾ����
 * @retval None
 */
void littlefs_demo(void)
{
  int err;
  lfs_file_t file;
  const char *test_data = "Hello LittleFS! This is a test file.";
  char read_buffer[100] = {0};
  lfs_size_t written = 0;    // Ԥ�ȳ�ʼ��
  lfs_ssize_t read_size = 0; // ʹ���з������ͽ��д�����

  printf("\r\n=== LittleFS Demo Start ===\r\n");

  // ��ʼ��LittleFS��ֲ��
  err = lfs_port_init();
  if (err < 0)
  {
    printf("LittleFS port init failed: %d\r\n", err);
    return;
  }
  printf("LittleFS port init success\r\n");

  // �����ļ�ϵͳ
  err = lfs_port_mount();
  if (err < 0)
  {
    printf("LittleFS mount failed: %d\r\n", err);
    return;
  }
  printf("LittleFS mount success\r\n");
  // ������д������ļ�
  err = lfs_file_open(&lfs_fs, &file, "test.txt", LFS_O_WRONLY | LFS_O_CREAT);
  if (err < 0)
  {
    printf("File open for write failed: %d\r\n", err);
    goto cleanup;
  }

  written = lfs_file_write(&lfs_fs, &file, test_data, strlen(test_data));
  if (written != strlen(test_data))
  {
    printf("File write failed: written %d, expected %d\r\n", (int)written, (int)strlen(test_data));
    lfs_file_close(&lfs_fs, &file);
    goto cleanup;
  }
  printf("File write success: %d bytes\r\n", (int)written);

  lfs_file_close(&lfs_fs, &file);

  // ��ȡ�����ļ�
  err = lfs_file_open(&lfs_fs, &file, "test.txt", LFS_O_RDONLY);
  if (err < 0)
  {
    printf("File open for read failed: %d\r\n", err);
    goto cleanup;
  }

  read_size = lfs_file_read(&lfs_fs, &file, read_buffer, sizeof(read_buffer) - 1);
  if (read_size < 0)
  {
    printf("File read failed: %d\r\n", (int)read_size);
    lfs_file_close(&lfs_fs, &file);
    goto cleanup;
  }

  read_buffer[read_size] = '\0'; // ȷ���ַ�������
  printf("File read success: %d bytes\r\n", (int)read_size);
  printf("File content: %s\r\n", read_buffer);

  lfs_file_close(&lfs_fs, &file);

  // ��֤����
  if (strcmp(test_data, read_buffer) == 0)
  {
    printf("Data verification success!\r\n");
  }
  else
  {
    printf("Data verification failed!\r\n");
  }
  // ��ȡ�ļ�ϵͳ״̬
  struct lfs_info info;
  err = lfs_stat(&lfs_fs, "test.txt", &info);
  if (err >= 0)
  {
    printf("File size: %d bytes\r\n", (int)info.size);
    printf("File type: %s\r\n", (info.type == LFS_TYPE_REG) ? "Regular file" : "Directory");
  }

  // �г���Ŀ¼�ļ�
  lfs_dir_t dir;
  err = lfs_dir_open(&lfs_fs, &dir, "/");
  if (err >= 0)
  {
    printf("Root directory contents:\r\n");
    while (true)
    {
      err = lfs_dir_read(&lfs_fs, &dir, &info);
      if (err < 0)
        break;
      if (err == 0)
        break; // û�и����ļ�      if (info.name[0] != '.')
      {        // ���� . �� .. Ŀ¼
        printf("  %s (%s, %d bytes)\r\n",
               info.name,
               (info.type == LFS_TYPE_REG) ? "file" : "dir",
               (int)info.size);
      }
    }
    lfs_dir_close(&lfs_fs, &dir);
  }

cleanup:
  printf("=== LittleFS Demo End ===\r\n\r\n");
}

void sfud_demo(uint32_t addr, size_t size, uint8_t *data)
{
  sfud_err result = SFUD_SUCCESS;
  extern sfud_flash *sfud_dev;
  const sfud_flash *flash = sfud_get_device(SFUD_W25_DEVICE_INDEX);
  size_t i;
  /* prepare write data */
  for (i = 0; i < size; i++)
  {
    data[i] = i;
  }
  /* erase test */
  result = sfud_erase(flash, addr, size);
  if (result == SFUD_SUCCESS)
  {
    printf("Erase the %s flash data finish. Start from 0x%08X, size is %zu.\r\n", flash->name, addr, size);
  }
  else
  {
    printf("Erase the %s flash data failed.\r\n", flash->name);
    return;
  }
  /* write test */
  result = sfud_write(flash, addr, size, data);
  if (result == SFUD_SUCCESS)
  {
    printf("Write the %s flash data finish. Start from 0x%08X, size is %zu.\r\n", flash->name, addr, size);
  }
  else
  {
    printf("Write the %s flash data failed.\r\n", flash->name);
    return;
  }
  /* read test */
  result = sfud_read(flash, addr, size, data);
  if (result == SFUD_SUCCESS)
  {
    printf("Read the %s flash data success. Start from 0x%08X, size is %zu. The data is:\r\n", flash->name, addr, size);
    printf("Offset (h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
    for (i = 0; i < size; i++)
    {
      if (i % 16 == 0)
      {
        printf("[%08X] ", addr + i);
      }
      printf("%02X ", data[i]);
      if (((i + 1) % 16 == 0) || i == size - 1)
      {
        printf("\r\n");
      }
    }
    printf("\r\n");
  }
  else
  {
    printf("Read the %s flash data failed.\r\n", flash->name);
  }
  /* data check */
  for (i = 0; i < size; i++)
  {
    if (data[i] != i % 256)
    {
      printf("Read and check write data has an error. Write the %s flash data failed.\r\n", flash->name);
      break;
    }
  }
  if (i == size)
  {
    printf("The %s flash test is success.\r\n", flash->name);
  }
}

/**
 * @brief ��ʼ��������LittleFS��SFUD
 *
 */
void initialize_and_test_LittleFS(void)
{
  if (sfud_init() == SFUD_SUCCESS)
  {
    sfud_demo(0, sizeof(sfud_demo_test_buf), sfud_demo_test_buf);

    // ����LittleFS��ʾ
    littlefs_demo();
  }
}
