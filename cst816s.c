/****************************************************************************
 * drivers/input/cst816s.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <poll.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>

#include <nuttx/input/cst816s.h>

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define CONFIG_INPUT_CYPRESS_CST816S_NPOLLWAITERS 1  ////  TODO

/* Register macros */

#define CST816S_SENSOR_EN                   0x0
#define CST816S_FSS_EN                      0x02
#define CST816S_TOGGLE_EN                   0x04
#define CST816S_LED_ON_EN                   0x06
#define CST816S_SENSITIVITY0                0x08
#define CST816S_SENSITIVITY1                0x09
#define CST816S_BASE_THRESHOLD0             0x0c
#define CST816S_BASE_THRESHOLD1             0x0d
#define CST816S_FINGER_THRESHOLD2           0x0e
#define CST816S_FINGER_THRESHOLD3           0x0f
#define CST816S_FINGER_THRESHOLD4           0x10
#define CST816S_FINGER_THRESHOLD5           0x11
#define CST816S_FINGER_THRESHOLD6           0x12
#define CST816S_FINGER_THRESHOLD7           0x13
#define CST816S_SENSOR_DEBOUNCE             0x1c
#define CST816S_BUTTON_HYS                  0x1d
#define CST816S_BUTTON_LBR                  0x1f
#define CST816S_BUTTON_NNT                  0x20
#define CST816S_BUTTON_NT                   0x21
#define CST816S_PROX_EN                     0x26
#define CST816S_PROX_CFG                    0x27
#define CST816S_PROX_CFG2                   0x28
#define CST816S_PROX_TOUCH_TH0              0x2a
#define CST816S_PROX_TOUCH_TH1              0x2c
#define CST816S_PROX_RESOLUTION0            0x2e
#define CST816S_PROX_RESOLUTION1            0x2f
#define CST816S_PROX_HYS                    0x30
#define CST816S_PROX_LBR                    0x32
#define CST816S_PROX_NNT                    0x33
#define CST816S_PROX_NT                     0x34
#define CST816S_PROX_POSITIVE_TH0           0x35
#define CST816S_PROX_POSITIVE_TH1           0x36
#define CST816S_PROX_NEGATIVE_TH0           0x39
#define CST816S_PROX_NEGATIVE_TH1           0x3a
#define CST816S_LED_ON_TIME                 0x3d
#define CST816S_BUZZER_CFG                  0x3e
#define CST816S_BUZZER_ON_TIME              0x3f
#define CST816S_GPO_CFG                     0x40
#define CST816S_PWM_DUTYCYCLE_CFG0          0x41
#define CST816S_PWM_DUTYCYCLE_CFG1          0x42
#define CST816S_PWM_DUTYCYCLE_CFG2          0x43
#define CST816S_PWM_DUTYCYCLE_CFG3          0x44
#define CST816S_SPO_CFG                     0x4c
#define CST816S_DEVICE_CFG0                 0x4d
#define CST816S_DEVICE_CFG1                 0x4e
#define CST816S_DEVICE_CFG2                 0x4f
#define CST816S_DEVICE_CFG3                 0x50
#define CST816S_I2C_ADDR                    0x51
#define CST816S_REFRESH_CTRL                0x52
#define CST816S_STATE_TIMEOUT               0x55
#define CST816S_CONFIG_CRC                  0x7e
#define CST816S_GPO_OUTPUT_STATE            0x80
#define CST816S_SENSOR_ID                   0x82
#define CST816S_CTRL_CMD                    0x86
#define CST816S_CTRL_CMD_STATUS             0x88
#define CST816S_CTRL_CMD_ERR                0x89
#define CST816S_SYSTEM_STATUS               0x8a
#define CST816S_PREV_CTRL_CMD_CODE          0x8c
#define CST816S_FAMILY_ID                   0x8f
#define CST816S_DEVICE_ID                   0x90
#define CST816S_DEVICE_REV                  0x92
#define CST816S_CALC_CRC                    0x94
#define CST816S_TOTAL_WORKING_SNS           0x97
#define CST816S_SNS_CP_HIGH                 0x98
#define CST816S_SNS_VDD_SHORT               0x9a
#define CST816S_SNS_GND_SHORT               0x9c
#define CST816S_SNS_SNS_SHORT               0x9e
#define CST816S_CMOD_SHIELD_TEST            0xa0
#define CST816S_BUTTON_STAT                 0xaa
#define CST816S_LATCHED_BUTTON_STAT         0xac
#define CST816S_PROX_STAT                   0xae
#define CST816S_LATCHED_PROX_STAT           0xaf
#define CST816S_SYNC_COUNTER0               0xb9
#define CST816S_DIFFERENCE_COUNT_SENSOR0    0xba
#define CST816S_DIFFERENCE_COUNT_SENSOR1    0xbc
#define CST816S_DIFFERENCE_COUNT_SENSOR2    0xbe
#define CST816S_DIFFERENCE_COUNT_SENSOR3    0xc0
#define CST816S_DIFFERENCE_COUNT_SENSOR4    0xc2
#define CST816S_DIFFERENCE_COUNT_SENSOR5    0xc4
#define CST816S_DIFFERENCE_COUNT_SENSOR6    0xc6
#define CST816S_DIFFERENCE_COUNT_SENSOR7    0xc8
#define CST816S_GPO_DATA                    0xda
#define CST816S_SYNC_COUNTER1               0xdb
#define CST816S_DEBUG_SENSOR_ID             0xdc
#define CST816S_DEBUG_CP                    0xdd
#define CST816S_DEBUG_DIFFERENCE_COUNT0     0xde
#define CST816S_DEBUG_BASELINE0             0xe0
#define CST816S_DEBUG_RAW_COUNT0            0xe2
#define CST816S_DEBUG_AVG_RAW_COUNT0        0xe4
#define CST816S_SYNC_COUNTER2               0xe7

/* Device commands for CST816S_CTRL_CMD */

#define CST816S_CMD_COMPLETED                               0
#define CST816S_CMD_CHECK_CONFIG_CRC                        2
#define CST816S_CMD_SET_CONFIG_CRC                          3
#define CST816S_CMD_ENTER_LOW_POWER_MODE                    7
#define CST816S_CMD_CLEAR_LATCHED                           8
#define CST816S_CMD_RESET_ADV_LOWPASS_FILTER_PROX_SENS_0    9
#define CST816S_CMD_RESET_ADV_LOWPASS_FILTER_PROX_SENS_1    10
#define CST816S_CMD_SOFTWARE_RESET                          255

#define CST816S_CMD_STATUS_SUCCESS                          0
#define CST816S_CMD_STATUS_ERROR                            1
#define CST816S_CMD_STATUS_MASK                             1

/* Completion times for device commands */

#define CST816S_CMD_MSECS_CHECK_CONFIG_CRC                  280 /* >220 (typ.) */
#define CST816S_CMD_MSECS_SOFTWARE_RESET                    50
#define CST816S_CMD_MSECS_CLEAR_LATCHED                     50

/* Other macros */

#define CST816S_I2C_RETRIES                 10
#define CST816S_NUM_SENSORS                 8
#define CST816S_EXPECTED_FAMILY_ID          0x9a
#define CST816S_EXPECTED_DEVICE_ID          0x0a03
#define CST816S_EXPECTED_DEVICE_REV         1
#define CST816S_SYNC_RETRIES                10

#ifndef CONFIG_CST816S_I2C_FREQUENCY
#  define CONFIG_CST816S_I2C_FREQUENCY      400000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cst816s_dev_s
{
  /* I2C bus and address for device. */

  struct i2c_master_s *i2c;
  uint8_t addr;

  /* Configuration for device. */

  ////TODO: struct cst816s_board_s *board;
  ////TODO: const struct cst816s_sensor_conf_s *sensor_conf;
  sem_t devsem;
  uint8_t cref;
  struct cst816s_debug_conf_s debug_conf;
  bool int_pending;

  struct pollfd *fds[CONFIG_INPUT_CYPRESS_CST816S_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int cst816s_open(FAR struct file *filep);
static int cst816s_close(FAR struct file *filep);
static ssize_t cst816s_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static ssize_t cst816s_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen);
static int cst816s_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_cst816s_fileops =
{
  cst816s_open,   /* open */
  cst816s_close,  /* close */
  cst816s_read,   /* read */
  cst816s_write,  /* write */
  NULL,           /* seek */
  NULL,           /* ioctl */
  cst816s_poll    /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL          /* unlink */
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cst816s_i2c_write(FAR struct cst816s_dev_s *dev, uint8_t reg,
                             const uint8_t *buf, size_t buflen)
{
  struct i2c_msg_s msgv[2] =
  {
    {
      .frequency = CONFIG_CST816S_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = 0,
      .buffer    = &reg,
      .length    = 1
    },
    {
      .frequency = CONFIG_CST816S_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = I2C_M_NOSTART,
      .buffer    = (void *)buf,
      .length    = buflen
    }
  };

  int ret = -EIO;
  int retries;

  /* ??? will respond with NACK to address when in low-power mode. Host
   * needs to retry address selection multiple times to get ??? to
   * wake-up.
   */

  for (retries = 0; retries < CST816S_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(dev->i2c, msgv, 2);
      if (ret == -ENXIO)
        {
          /* -ENXIO is returned when getting NACK from response.
           * Keep trying.
           */

          continue;
        }

      if (ret >= 0)
        {
          /* Success! */

          return 0;
        }
    }

  /* Failed to read sensor. */

  return ret;
}

static int cst816s_i2c_read(FAR struct cst816s_dev_s *dev, uint8_t reg,
                            uint8_t *buf, size_t buflen)
{
  struct i2c_msg_s msgv[2] =
  {
    {
      .frequency = CONFIG_CST816S_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = 0,
      .buffer    = &reg,
      .length    = 1
    },
    {
      .frequency = CONFIG_CST816S_I2C_FREQUENCY,
      .addr      = dev->addr,
      .flags     = I2C_M_READ,
      .buffer    = buf,
      .length    = buflen
    }
  };

  int ret = -EIO;
  int retries;

  /* ??? will respond with NACK to address when in low-power mode. Host
   * needs to retry address selection multiple times to get ??? to
   * wake-up.
   */

  for (retries = 0; retries < CST816S_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(dev->i2c, msgv, 2);
      if (ret == -ENXIO)
        {
          /* -ENXIO is returned when getting NACK from response.
           * Keep trying.
           */

          continue;
        }
      else if (ret >= 0)
        {
          /* Success! */

          return 0;
        }
      else
        {
          /* Some other error. Try to reset I2C bus and keep trying. */

#ifdef CONFIG_I2C_RESET
          if (retries == CST816S_I2C_RETRIES - 1)
            {
              break;
            }

          ret = I2C_RESET(dev->i2c);
          if (ret < 0)
            {
              iinfo("I2C_RESET failed: %d\n", ret);
              return ret;
            }
#endif
        }
    }

  /* Failed to read sensor. */

  return ret;
}

static int cst816s_check_cmd_status(FAR struct cst816s_dev_s *dev)
{
  const uint8_t start_reg = CST816S_CTRL_CMD;
  const uint8_t last_reg = CST816S_CTRL_CMD_ERR;
  uint8_t readbuf[CST816S_CTRL_CMD_ERR - CST816S_CTRL_CMD + 1];
  uint8_t cmd;
  uint8_t cmd_status;
  uint8_t cmd_err;
  int ret;

  DEBUGASSERT(last_reg - start_reg + 1 == sizeof(readbuf));

  /* Multi-byte read to get command status. */

  ret = cst816s_i2c_read(dev, start_reg, readbuf, sizeof(readbuf));
  if (ret < 0)
    {
      iinfo("cmd status get failed. ret=%d\n", ret);
      return ret;
    }

  cmd        = readbuf[CST816S_CTRL_CMD - CST816S_CTRL_CMD];
  cmd_status = readbuf[CST816S_CTRL_CMD_STATUS - CST816S_CTRL_CMD];
  cmd_err    = readbuf[CST816S_CTRL_CMD_ERR - CST816S_CTRL_CMD];

  iinfo("cmd: %d, status: %d, err: %d\n", cmd, cmd_status, cmd_err);

  if (cmd != CST816S_CMD_COMPLETED)
    {
      return -EBUSY;
    }

  if ((cmd_status & CST816S_CMD_STATUS_MASK) == CST816S_CMD_STATUS_SUCCESS)
    {
      /* Success. */

      return 0;
    }

  return cmd_err;
}

static int cst816s_save_check_crc(FAR struct cst816s_dev_s *dev)
{
  uint8_t reg = CST816S_CTRL_CMD;
  uint8_t cmd = CST816S_CMD_CHECK_CONFIG_CRC;
  int ret;

  ret = cst816s_i2c_write(dev, reg, &cmd, 1);
  if (ret < 0)
    {
      iinfo("CST816S_CTRL_CMD:CHECK_CONFIG_CRC write failed.\n");
      return ret;
    }

  nxsig_usleep(CST816S_CMD_MSECS_CHECK_CONFIG_CRC * 1000);

  ret = cst816s_check_cmd_status(dev);
  if (ret != 0)
    {
      return ret < 0 ? ret : -EIO;
    }

  return 0;
}

static int cst816s_software_reset(FAR struct cst816s_dev_s *dev)
{
  uint8_t reg = CST816S_CTRL_CMD;
  uint8_t cmd = CST816S_CMD_SOFTWARE_RESET;
  int ret;

  ret = cst816s_i2c_write(dev, reg, &cmd, 1);
  if (ret < 0)
    {
      iinfo("CST816S_CTRL_CMD:SOFTWARE_RESET write failed.\n");
      return ret;
    }

  nxsig_usleep(CST816S_CMD_MSECS_SOFTWARE_RESET * 1000);

  ret = cst816s_check_cmd_status(dev);
  if (ret != 0)
    {
      return ret < 0 ? ret : -EIO;
    }

  return 0;
}

static int cst816s_enter_low_power_mode(FAR struct cst816s_dev_s *dev)
{
  uint8_t reg = CST816S_CTRL_CMD;
  uint8_t cmd = CST816S_CMD_ENTER_LOW_POWER_MODE;
  int ret;

  ret = cst816s_i2c_write(dev, reg, &cmd, 1);
  if (ret < 0)
    {
      iinfo("CST816S_CTRL_CMD:SOFTWARE_RESET write failed.\n");
      return ret;
    }

  /* Device is now in low-power mode and not scanning. Further communication
   * will cause wake-up and make chip resume scanning operations.
   */

  return 0;
}

static int cst816s_clear_latched(FAR struct cst816s_dev_s *dev)
{
  uint8_t reg = CST816S_CTRL_CMD;
  uint8_t cmd = CST816S_CMD_CLEAR_LATCHED;
  int ret;

  ret = cst816s_i2c_write(dev, reg, &cmd, 1);
  if (ret < 0)
    {
      iinfo("CST816S_CTRL_CMD:  "
                  "CST816S_CMD_CLEAR_LATCHED write failed.\n");
      return ret;
    }

  nxsig_usleep(CST816S_CMD_MSECS_CLEAR_LATCHED * 1000);

  ret = cst816s_check_cmd_status(dev);
  if (ret != 0)
    {
      return ret < 0 ? ret : -EIO;
    }

  return 0;
}

static int cst816s_debug_setup(FAR struct cst816s_dev_s *dev,
                               FAR const struct cst816s_debug_conf_s *conf)
{
  uint8_t reg = CST816S_SENSOR_ID;
  int ret;

  /* Store new debug configuration. */

  dev->debug_conf = *conf;

  if (!conf->debug_mode)
    {
      return 0;
    }

  /* Setup debug sensor id. */

  ret = cst816s_i2c_write(dev, reg, &conf->debug_sensor_id, 1);
  if (ret < 0)
    {
      iinfo("CST816S_SENSOR_ID write failed.\n");

      dev->debug_conf.debug_mode = false;
    }

  return ret;
}

static int
  cst816s_device_configuration(FAR struct cst816s_dev_s *dev,
                               FAR const struct cst816s_sensor_conf_s *conf)
{
  const uint8_t start_reg = CST816S_SENSOR_EN;
  const uint8_t last_reg = CST816S_CONFIG_CRC + 1;
  uint8_t value;
  int ret = 0;

  DEBUGASSERT(sizeof(conf->conf_data) == last_reg - start_reg + 1);

  ret = cst816s_i2c_read(dev, CST816S_CTRL_CMD, &value, 1);
  if (ret < 0)
    {
      iinfo("CST816S_CTRL_CMD read failed.\n");
      return ret;
    }

  if (value != CST816S_CMD_COMPLETED)
    {
      /* Device is busy processing previous command. */

      return -EBUSY;
    }

  ret = cst816s_i2c_write(dev, start_reg, conf->conf_data,
                          last_reg - start_reg + 1);
  if (ret < 0)
    {
      iinfo("configuration write failed.\n");
      return ret;
    }

  ret = cst816s_save_check_crc(dev);
  if (ret < 0)
    {
      iinfo("save check CRC failed. ret=%d\n", ret);
      return ret;
    }

  ret = cst816s_software_reset(dev);
  if (ret < 0)
    {
      iinfo("software reset failed.\n");
      return ret;
    }

  ////TODO: dev->board->irq_enable(dev->board, true);

  return 0;
}

static int cst816s_get_sensor_status(FAR struct cst816s_dev_s *dev,
                                     FAR void *buf)
{
  struct cst816s_sensor_status_s status =
  {
  };

  const uint8_t start_reg = CST816S_BUTTON_STAT;
  const uint8_t last_reg = CST816S_LATCHED_PROX_STAT;
  uint8_t readbuf[CST816S_LATCHED_PROX_STAT - CST816S_BUTTON_STAT + 1];
  int ret;

  DEBUGASSERT(last_reg - start_reg + 1 == sizeof(readbuf));

  /* Attempt to sensor status registers. */

  ret = cst816s_i2c_read(dev, start_reg, readbuf, sizeof(readbuf));
  if (ret < 0)
    {
      iinfo("Sensor status read failed.\n");

      return ret;
    }

  status.button            =
    (readbuf[CST816S_BUTTON_STAT + 0 - start_reg]) |
    (readbuf[CST816S_BUTTON_STAT + 1 - start_reg] << 8);
  status.proximity         =
    readbuf[CST816S_PROX_STAT - start_reg];

  status.latched_button    =
    (readbuf[CST816S_LATCHED_BUTTON_STAT + 0 - start_reg]) |
    (readbuf[CST816S_LATCHED_BUTTON_STAT + 1 - start_reg] << 8);
  status.latched_proximity =
    readbuf[CST816S_LATCHED_PROX_STAT - start_reg];

  memcpy(buf, &status, sizeof(status));

  iinfo("but: %x, prox: %x; latched[btn: %x, prox: %x]\n",
              status.button, status.proximity, status.latched_button,
              status.latched_button);

  return 0;
}

static int cst816s_get_sensor_debug_data(FAR struct cst816s_dev_s *dev,
                                         FAR void *buf)
{
  struct cst816s_sensor_debug_s data =
  {
  };

  const uint8_t start_reg = CST816S_SYNC_COUNTER1;
  const uint8_t last_reg = CST816S_SYNC_COUNTER2;
  uint8_t readbuf[CST816S_SYNC_COUNTER2 - CST816S_SYNC_COUNTER1 + 1];
  uint8_t sync1;
  uint8_t sync2;
  int ret;
  int retries;

  DEBUGASSERT(last_reg - start_reg + 1 == sizeof(readbuf));

  for (retries = CST816S_SYNC_RETRIES; retries > 0; retries--)
    {
      ret = cst816s_i2c_read(dev, start_reg, readbuf, sizeof(readbuf));
      if (ret < 0)
        {
          iinfo("Sensor debug data read failed.\n");

          return ret;
        }

      /* Sync counters need to match. */

      sync1 = readbuf[CST816S_SYNC_COUNTER1 - start_reg];
      sync2 = readbuf[CST816S_SYNC_COUNTER2 - start_reg];

      if (sync1 == sync2)
        {
          break;
        }
    }

  if (retries == 0)
    {
      return -EIO;
    }

  data.sensor_average_counts =
      (readbuf[CST816S_DEBUG_AVG_RAW_COUNT0 + 0 - start_reg]) |
      (readbuf[CST816S_DEBUG_AVG_RAW_COUNT0 + 1 - start_reg] << 8);
  data.sensor_baseline =
      (readbuf[CST816S_DEBUG_BASELINE0 + 0 - start_reg]) |
      (readbuf[CST816S_DEBUG_BASELINE0 + 1 - start_reg] << 8);
  data.sensor_diff_counts =
      (readbuf[CST816S_DEBUG_DIFFERENCE_COUNT0 + 0 - start_reg]) |
      (readbuf[CST816S_DEBUG_DIFFERENCE_COUNT0 + 1 - start_reg] << 8);
  data.sensor_raw_counts =
      (readbuf[CST816S_DEBUG_RAW_COUNT0 + 0 - start_reg]) |
      (readbuf[CST816S_DEBUG_RAW_COUNT0 + 1 - start_reg] << 8);
  data.sensor_total_capacitance = readbuf[CST816S_DEBUG_CP - start_reg];

  memcpy(buf, &data, sizeof(data));

  iinfo("avg_cnt: %d, baseline: %d, diff_cnt: %d, raw_cnt: %d, "
              "total_cp: %d\n",
              data.sensor_average_counts, data.sensor_baseline,
              data.sensor_diff_counts, data.sensor_raw_counts,
              data.sensor_total_capacitance);

  return 0;
}

static int cst816s_probe_device(FAR struct cst816s_dev_s *dev)
{
  const uint8_t start_reg = CST816S_FAMILY_ID;
  const uint8_t last_reg = CST816S_DEVICE_REV;
  uint8_t readbuf[CST816S_DEVICE_REV - CST816S_FAMILY_ID + 1];
  uint8_t fam_id;
  uint16_t dev_id;
  uint8_t dev_rev;
  int ret;

  DEBUGASSERT(last_reg - start_reg + 1 == sizeof(readbuf));

  /* Attempt to read device identification registers with multi-byte
   * read.
   */

  ret = cst816s_i2c_read(dev, start_reg, readbuf, sizeof(readbuf));
  if (ret < 0)
    {
      /* Failed to read registers from device. */

      iinfo("Probe failed.\n");

      return ret;
    }

  /* Check result. */

  fam_id = readbuf[CST816S_FAMILY_ID - start_reg];
  dev_id = (readbuf[CST816S_DEVICE_ID + 0 - start_reg]) |
           (readbuf[CST816S_DEVICE_ID + 1 - start_reg] << 8);
  dev_rev = readbuf[CST816S_DEVICE_REV - start_reg];

  iinfo("family_id: 0x%02x, device_id: 0x%04x, device_rev: %d\n",
              fam_id, dev_id, dev_rev);

  if (fam_id != CST816S_EXPECTED_FAMILY_ID ||
      dev_id != CST816S_EXPECTED_DEVICE_ID ||
      dev_rev != CST816S_EXPECTED_DEVICE_REV)
    {
      iinfo("Probe failed, dev-id mismatch!\n");
      iinfo("  Expected: family_id: 0x%02x, device_id: "
                  "0x%04x, device_rev: %d\n",
                  CST816S_EXPECTED_FAMILY_ID,
                  CST816S_EXPECTED_DEVICE_ID,
                  CST816S_EXPECTED_DEVICE_REV);

      return -ENXIO;
    }

  return 0;
}

static ssize_t cst816s_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode;
  FAR struct cst816s_dev_s *priv;
  size_t outlen;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EINVAL;

  if (priv->debug_conf.debug_mode)
    {
      outlen = sizeof(struct cst816s_sensor_debug_s);
      if (buflen >= outlen)
        {
          ret = cst816s_get_sensor_debug_data(priv, buffer);
        }
    }
  else
    {
      outlen = sizeof(struct cst816s_sensor_status_s);
      if (buflen >= outlen)
        {
          ret = cst816s_get_sensor_status(priv, buffer);
        }
    }

  flags = enter_critical_section();
  priv->int_pending = false;
  leave_critical_section(flags);

  nxsem_post(&priv->devsem);
  return ret < 0 ? ret : outlen;
}

static ssize_t cst816s_write(FAR struct file *filep, FAR const char *buffer,
                             size_t buflen)
{
  FAR struct inode *inode;
  FAR struct cst816s_dev_s *priv;
  enum cst816s_cmd_e type;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  if (buflen < sizeof(enum cst816s_cmd_e))
    {
      return -EINVAL;
    }

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  type = *(FAR const enum cst816s_cmd_e *)buffer;

  switch (type)
    {
    case CYPRESS_CST816S_CMD_SENSOR_CONF:
      {
        FAR const struct cst816s_cmd_sensor_conf_s *conf =
            (FAR const struct cst816s_cmd_sensor_conf_s *)buffer;

        if (buflen != sizeof(*conf))
          {
            ret = -EINVAL;
            goto out;
          }

        ret = cst816s_device_configuration(priv, &conf->conf);
        break;
      }

    case CYPRESS_CST816S_CMD_DEBUG_CONF:
      {
        FAR const struct cst816s_cmd_debug_conf_s *conf =
            (FAR const struct cst816s_cmd_debug_conf_s *)buffer;

        if (buflen != sizeof(*conf))
          {
            ret = -EINVAL;
            goto out;
          }

        ret = cst816s_debug_setup(priv, &conf->conf);
        break;
      }

    case CYPRESS_CST816S_CMD_CLEAR_LATCHED:
      {
        if (buflen != sizeof(type))
          {
            ret = -EINVAL;
            goto out;
          }

        ret = cst816s_clear_latched(priv);
        break;
      }

    default:
      ret = -EINVAL;
      break;
    }

out:
  nxsem_post(&priv->devsem);

  return ret < 0 ? ret : buflen;
}

static int cst816s_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct cst816s_dev_s *priv;
  unsigned int use_count;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  use_count = priv->cref + 1;
  if (use_count == 1)
    {
      /* First user, do power on. */

#ifdef TODO ////
      ret = priv->board->set_power(priv->board, true);
      if (ret < 0)
        {
          goto out_sem;
        }
#endif  ////  TODO

      /* Let chip to power up before probing */

      nxsig_usleep(100 * 1000);

      /* Check that device exists on I2C. */

      ret = cst816s_probe_device(priv);
      if (ret < 0)
        {
          /* No such device. Power off the switch. */

          ////TODO: priv->board->set_power(priv->board, false);
          goto out_sem;
        }

#ifdef TODO ////
      if (priv->sensor_conf)
        {
          /* Do configuration. */

          ret = cst816s_device_configuration(priv, priv->sensor_conf);
          if (ret < 0)
            {
              /* Configuration failed. Power off the switch. */

              priv->board->set_power(priv->board, false);
              goto out_sem;
            }
        }
#endif  ////  TODO

      priv->cref = use_count;
    }
  else
    {
      DEBUGASSERT(use_count < UINT8_MAX && use_count > priv->cref);

      priv->cref = use_count;
      ret = 0;
    }

out_sem:
  nxsem_post(&priv->devsem);
  return ret;
}

static int cst816s_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct cst816s_dev_s *priv;
  int use_count;
  int ret;

  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  use_count = priv->cref - 1;
  if (use_count == 0)
    {
      /* Disable interrupt */

      ////TODO: priv->board->irq_enable(priv->board, false);

      /* Set chip in low-power mode. */

      cst816s_enter_low_power_mode(priv);

      /* Last user, do power off. */

      ////TODO: priv->board->set_power(priv->board, false);

      priv->debug_conf.debug_mode = false;
      priv->cref = use_count;
    }
  else
    {
      DEBUGASSERT(use_count > 0);

      priv->cref = use_count;
    }

  nxsem_post(&priv->devsem);

  return 0;
}

static void cst816s_poll_notify(FAR struct cst816s_dev_s *priv)
{
  int i;

  DEBUGASSERT(priv != NULL);

  for (i = 0; i < CONFIG_INPUT_CYPRESS_CST816S_NPOLLWAITERS; i++)
    {
      struct pollfd *fds = priv->fds[i];
      if (fds)
        {
          iinfo("Report events: %02x\n", fds->revents);

          fds->revents |= POLLIN;
          nxsem_post(fds->sem);
        }
    }
}

static int cst816s_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct cst816s_dev_s *priv;
  FAR struct inode *inode;
  bool pending;
  int ret = 0;
  int i;

  DEBUGASSERT(filep && fds);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = (FAR struct cst816s_dev_s *)inode->i_private;

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      /* Ignore waits that do not include POLLIN */

      if ((fds->events & POLLIN) == 0)
        {
          ret = -EDEADLK;
          goto out;
        }

      /* This is a request to set up the poll.  Find an available slot for
       * the poll structure reference.
       */

      for (i = 0; i < CONFIG_INPUT_CYPRESS_CST816S_NPOLLWAITERS; i++)
        {
          /* Find an available slot */

          if (!priv->fds[i])
            {
              /* Bind the poll structure and this slot */

              priv->fds[i] = fds;
              fds->priv = &priv->fds[i];
              break;
            }
        }

      if (i >= CONFIG_INPUT_CYPRESS_CST816S_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
        }
      else
        {
          pending = priv->int_pending;
          if (pending)
            {
              cst816s_poll_notify(priv);
            }
        }
    }
  else if (fds->priv)
    {
      /* This is a request to tear down the poll. */

      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;
      DEBUGASSERT(slot != NULL);

      /* Remove all memory of the poll setup */

      *slot = NULL;
      fds->priv = NULL;
    }

out:
  nxsem_post(&priv->devsem);
  return ret;
}

static int cst816s_isr_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct cst816s_dev_s *priv = (FAR struct cst816s_dev_s *)arg;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  flags = enter_critical_section();
  priv->int_pending = true;
  leave_critical_section(flags);

  cst816s_poll_notify(priv);
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int cst816s_register(FAR const char *devpath,
                             FAR struct i2c_master_s *i2c_dev,
                             uint8_t i2c_devaddr)
{
  struct cst816s_dev_s *priv;
  int ret = 0;

  /* Allocate device private structure. */

  priv = kmm_zalloc(sizeof(struct cst816s_dev_s));
  if (!priv)
    {
      iinfo("Memory cannot be allocated for driver\n");
      return -ENOMEM;
    }

  /* Setup device structure. */

  priv->addr = i2c_devaddr;
  priv->i2c = i2c_dev;

  nxsem_init(&priv->devsem, 0, 1);

  ret = register_driver(devpath, &g_cst816s_fileops, 0666, priv);
  if (ret < 0)
    {
      kmm_free(priv);
      iinfo("Error occurred during the driver registration\n");
      return ret;
    }

  iinfo("Registered with %d\n", ret);

  /* Prepare interrupt line and handler. */

  ////TODO: priv->board->irq_attach(priv->board, cst816s_isr_handler, priv);
  ////TODO: priv->board->irq_enable(priv->board, false);

  return 0;
}
