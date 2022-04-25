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

#include <nuttx/input/touchscreen.h>
#include <nuttx/input/cst816s.h>
#include <nuttx/ioexpander/gpio.h>
#include <nuttx/ioexpander/bl602_expander.h>

#include "../arch/risc-v/src/bl602/bl602_gpio.h"
#include "../boards/risc-v/bl602/bl602evb/include/board.h"

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define CONFIG_INPUT_CYPRESS_CST816S_NPOLLWAITERS 10

/* CST816S Registers for Touch Data and Chip ID */

#define CST816S_REG_TOUCHDATA 0x00
#define CST816S_REG_CHIPID    0xA7

/* I2C Retries and Frequency */

#define CST816S_I2C_RETRIES                 10

#ifndef CONFIG_CST816S_I2C_FREQUENCY
#  define CONFIG_CST816S_I2C_FREQUENCY      400000
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CST816S Device */

struct cst816s_dev_s
{
  /* I2C bus and address for device. */

  struct i2c_master_s *i2c;
  uint8_t addr;

  /* Configuration for device. */

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
static int cst816s_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* File Operations exposed to NuttX Apps */

static const struct file_operations g_cst816s_fileops =
{
  cst816s_open,   /* open */
  cst816s_close,  /* close */
  cst816s_read,   /* read */
  NULL,           /* write */
  NULL,           /* seek */
  NULL,           /* ioctl */
  cst816s_poll    /* poll */
#ifndef CONFIG_DISABLE_PSEUDOFS_OPERATIONS
  , NULL          /* unlink */
#endif
};

/* Last event, last ID and last valid touch coordinates */

static uint8_t  last_event = 0xff;
static uint8_t  last_id    = 0xff;
static uint16_t last_x     = 0xffff;
static uint16_t last_y     = 0xffff;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: cst816s_i2c_read
 *
 * Description:
 *   Read from I2C device.
 *
 ****************************************************************************/

static int cst816s_i2c_read(FAR struct cst816s_dev_s *dev, uint8_t reg,
                            uint8_t *buf, size_t buflen)
{
  int ret = -EIO;
  int retries;
  struct i2c_msg_s msgv[2] =
  {
    {
      .frequency = CONFIG_CST816S_I2C_FREQUENCY,
      .addr      = dev->addr,
#ifdef CONFIG_BL602_I2C0
      .flags     = I2C_M_NOSTART,  /* BL602 must send Register ID as Sub Address */
#else
      .flags     = 0,  /* Otherwise send Register ID normally */
#endif /* CONFIG_BL602_I2C0 */
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

  /* CST816S will respond with NACK to address when in low-power mode. Host
   * needs to retry address selection multiple times to get CST816S to
   * wake-up.
   */

  iinfo("\n");
  for (retries = 0; retries < CST816S_I2C_RETRIES; retries++)
    {
      ret = I2C_TRANSFER(dev->i2c, msgv, 2);
      if (ret == -ENXIO)
        {
          /* -ENXIO is returned when getting NACK from response.
           * Keep trying.
           */

          iwarn("I2C NACK\n");
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

          iwarn("I2C error\n");
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

/****************************************************************************
 * Name: cst816s_get_touch_data
 *
 * Description:
 *   Read Touch Data over I2C.
 *
 ****************************************************************************/

static int cst816s_get_touch_data(FAR struct cst816s_dev_s *dev, FAR void *buf)
{
  struct touch_sample_s data;
  uint8_t readbuf[7];
  int ret;

  /* Read the raw touch data. */

  iinfo("\n");
  ret = cst816s_i2c_read(dev, CST816S_REG_TOUCHDATA, readbuf, sizeof(readbuf));
  if (ret < 0)
    {
      iinfo("Read touch data failed\n");
      return ret;
    }

  /* Interpret the raw touch data. */

  uint8_t id = readbuf[5] >> 4;
  uint8_t touchpoints = readbuf[2] & 0x0f;
  uint8_t xhigh = readbuf[3] & 0x0f;
  uint8_t xlow  = readbuf[4];
  uint8_t yhigh = readbuf[5] & 0x0f;
  uint8_t ylow  = readbuf[6];
  uint8_t event = readbuf[3] >> 6;  /* 0 = Touch Down, 1 = Touch Up, 2 = Contact */
  uint16_t x  = (xhigh  << 8) | xlow;
  uint16_t y  = (yhigh  << 8) | ylow;

  /* If touch coordinates are invalid, return the last valid coordinates. */

  bool valid = true;
  if (x >= 240 || y >= 240)
    {
      iwarn("Invalid touch data: id=%d, touch=%d, x=%d, y=%d\n", id, touchpoints, x, y);
      if (last_event == 0xff)  /* Quit if we have no last valid coordinates. */
        {
          ierr("Can't return touch data: id=%d, touch=%d, x=%d, y=%d\n", id, touchpoints, x, y);
          return -EINVAL;
        }
      valid = false;
      id = last_id;
      x  = last_x;
      y  = last_y;
    }

  /* Remember the last valid touch data. */

  last_event = event;
  last_id    = id;
  last_x     = x;
  last_y     = y;

  /* Set the touch data fields. */

  memset(&data, 0, sizeof(data));
  data.npoints     = 1;
  data.point[0].id = id;
  data.point[0].x  = x;
  data.point[0].y  = y;

  /* Set the touch flags. */

  if (event == 0)  /* Touch Down */
    {
      iinfo("DOWN: id=%d, touch=%d, x=%d, y=%d\n", id, touchpoints, x, y);
      if (valid)  /* Touch coordinates were valid. */
        {
          data.point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID;
        }
      else  /* Touch coordinates were invalid. */
        {
          data.point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID;
        }
    }
  else if (event == 1)  /* Touch Up */
    {
      iinfo("UP: id=%d, touch=%d, x=%d, y=%d\n", id, touchpoints, x, y);
      if (valid)  /* Touch coordinates were valid. */
        {
          data.point[0].flags  = TOUCH_UP | TOUCH_ID_VALID | TOUCH_POS_VALID;
        }
      else  /* Touch coordinates were invalid. */
        {
          data.point[0].flags  = TOUCH_UP | TOUCH_ID_VALID;
        }
    }
  else  /* Reject Contact */
    {
      iinfo("CONTACT: id=%d, touch=%d, x=%d, y=%d\n", id, touchpoints, x, y);
      return -EINVAL;
    }

  /* Return the touch data. */

  memcpy(buf, &data, sizeof(data));

  iinfo("  id:      %d\n",   data.point[0].id);
  iinfo("  flags:   %02x\n", data.point[0].flags);
  iinfo("  x:       %d\n",   data.point[0].x);
  iinfo("  y:       %d\n",   data.point[0].y);

  return sizeof(data);
}

/****************************************************************************
 * Name: cst816s_read
 *
 * Description:
 *   Read Touch Data from the device.
 *
 ****************************************************************************/

static ssize_t cst816s_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  FAR struct inode *inode;
  FAR struct cst816s_dev_s *priv;
  size_t outlen;
  irqstate_t flags;
  int ret;

  DEBUGASSERT(buffer);
  DEBUGASSERT(buflen > 0);
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  /* Wait for semaphore to prevent concurrent reads */

  ret = nxsem_wait(&priv->devsem);
  if (ret < 0)
    {
      return ret;
    }

  ret = -EINVAL;

  /* Read the touch data, only if screen has been touched or if we're waiting for touch up */

  outlen = sizeof(struct touch_sample_s);
  if ((priv->int_pending || last_event == 0) && buflen >= outlen)
    {
      ret = cst816s_get_touch_data(priv, buffer);
    }

  /* Clear pending flag with critical section */

  flags = enter_critical_section();
  priv->int_pending = false;
  leave_critical_section(flags);

  /* Release semaphore and allow next read */

  nxsem_post(&priv->devsem);
  return ret < 0 ? ret : outlen;
}

/****************************************************************************
 * Name: cst816s_open
 *
 * Description:
 *   Open the device.
 *
 ****************************************************************************/

static int cst816s_open(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct cst816s_dev_s *priv;
  unsigned int use_count;
  int ret;

  iinfo("\n");
  DEBUGASSERT(filep);
  inode = filep->f_inode;

  DEBUGASSERT(inode && inode->i_private);
  priv = inode->i_private;

  /* Wait for semaphore */

  ret = nxsem_wait_uninterruptible(&priv->devsem);
  if (ret < 0)
    {
      ierr("Semaphore failed\n");
      return ret;
    }

  use_count = priv->cref + 1;
  DEBUGASSERT(use_count < UINT8_MAX && use_count > priv->cref);

  priv->cref = use_count;
  ret = 0;

  /* Release semaphore */

  nxsem_post(&priv->devsem);
  return ret;
}

/****************************************************************************
 * Name: cst816s_close
 *
 * Description:
 *   Close the device.
 *
 ****************************************************************************/

static int cst816s_close(FAR struct file *filep)
{
  FAR struct inode *inode;
  FAR struct cst816s_dev_s *priv;
  int use_count;
  int ret;

  iinfo("\n");
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

/****************************************************************************
 * Name: cst816s_poll_notify
 *
 * Description:
 *   Notify all waiting pollers.
 *
 ****************************************************************************/

static void cst816s_poll_notify(FAR struct cst816s_dev_s *priv)
{
  int i;

  iinfo("\n");
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

/****************************************************************************
 * Name: cst816s_poll
 *
 * Description:
 *   Poll for updates.
 *
 ****************************************************************************/

static int cst816s_poll(FAR struct file *filep, FAR struct pollfd *fds,
                        bool setup)
{
  FAR struct cst816s_dev_s *priv;
  FAR struct inode *inode;
  bool pending;
  int ret = 0;
  int i;

  iinfo("\n");
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

/****************************************************************************
 * Name: cst816s_isr_handler
 *
 * Description:
 *   Handle GPIO Interrupt triggered by touch.
 *
 ****************************************************************************/

static int cst816s_isr_handler(FAR struct ioexpander_dev_s *dev,
                               ioe_pinset_t pinset, FAR void *arg)
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

/****************************************************************************
 * Name: cst816s_register
 *
 * Description:
 *   Register the CST816S device (e.g. /dev/input0).
 *
 ****************************************************************************/

int cst816s_register(FAR const char *devpath,
                     FAR struct i2c_master_s *i2c_dev,
                     uint8_t i2c_devaddr)
{
  uint8_t gpio_pin = (BOARD_TOUCH_INT & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  struct cst816s_dev_s *priv;
  void *handle;
  int ret = 0;

  /* Allocate device private structure. */

  iinfo("path=%s, addr=%d\n", devpath, i2c_devaddr);
  priv = kmm_zalloc(sizeof(struct cst816s_dev_s));
  if (!priv)
    {
      ierr("Memory allocation failed\n");
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
      ierr("Driver registration failed\n");
      return ret;
    }

  /* Configure GPIO interrupt to be triggered on falling edge. */

  DEBUGASSERT(bl602_expander != NULL);
  IOEXP_SETOPTION(bl602_expander, gpio_pin, IOEXPANDER_OPTION_INTCFG,
                  (FAR void *)IOEXPANDER_VAL_FALLING);

  /* Attach GPIO interrupt handler. */

  handle = IOEP_ATTACH(bl602_expander,
                       (ioe_pinset_t)1 << gpio_pin,
                       cst816s_isr_handler,
                       priv);
  if (handle == NULL)
    {
      kmm_free(priv);
      ierr("Attach interrupt failed\n");
      return -EIO;
    }

  iinfo("Driver registered\n");
  return 0;
}
