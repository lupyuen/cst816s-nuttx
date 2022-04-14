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

#include "../arch/risc-v/src/bl602/bl602_gpio.h"  ////  TODO
#include "../boards/risc-v/bl602/bl602evb/include/board.h"  ////  TODO

/****************************************************************************
 * Pre-Processor Definitions
 ****************************************************************************/

#define CONFIG_INPUT_CYPRESS_CST816S_NPOLLWAITERS 10  ////  TODO

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

/* Interrupt Handler */

typedef int isr_handler(int irq, FAR void *context, FAR void *arg);

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
static int bl602_irq_attach(gpio_pinset_t pinset, FAR isr_handler *callback, FAR void *arg);
static int bl602_irq_enable(bool enable);

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
  iinfo("\n"); ////
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

  int ret = -EIO;
  int retries;

  /* CST816S will respond with NACK to address when in low-power mode. Host
   * needs to retry address selection multiple times to get CST816S to
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

/****************************************************************************
 * Name: cst816s_get_touch_data
 *
 * Description:
 *   Read Touch Data over I2C.
 *
 ****************************************************************************/

static int cst816s_get_touch_data(FAR struct cst816s_dev_s *dev, FAR void *buf)
{
  iinfo("\n"); ////
  struct touch_sample_s data;
  uint8_t readbuf[7];
  int ret;

  /* Read the raw touch data. */

  ret = cst816s_i2c_read(dev, CST816S_REG_TOUCHDATA, readbuf, sizeof(readbuf));
  if (ret < 0)
    {
      iinfo("Read touch data failed\n");
      return ret;
    }

  /* Interpret the raw touch data. */

  uint8_t idhigh = readbuf[0] & 0x0f;
  uint8_t idlow  = readbuf[1];
  uint8_t touchpoints = readbuf[2] & 0x0f;  /* Touch Points can only be 0 or 1. */
  uint8_t xhigh = readbuf[3] & 0x0f;
  uint8_t xlow  = readbuf[4];
  uint8_t yhigh = readbuf[5] & 0x0f;
  uint8_t ylow  = readbuf[6];
  uint16_t id = (idhigh << 8) | idlow;
  uint16_t x  = (xhigh  << 8) | xlow;
  uint16_t y  = (yhigh  << 8) | ylow;

  /* Validate the touch coordinates. */

  bool valid = true;
  if (x >= 240 || y >= 240) {
    valid = false;
    return -EINVAL;  /* Must not return invalid coordinates, because lvgldemo can't handle. */
    //  iwarn("Invalid touch data: x=%d, y=%d\n", x, y);
  }

  /* Set the touch data fields. */

  memset(&data, 0, sizeof(data));
  data.npoints     = 1;
  data.point[0].id = id;
  data.point[0].x  = x;
  data.point[0].y  = y;

  /* Set the touch flags. */

  if (touchpoints > 0)  /* Panel was touched. */
    {
      if (valid)  /* Touch coordinates were valid. */
        {
          data.point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID;
        }
      else  /* Touch coordinates were invalid. */
        {
          data.point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID;
        }
    }
  else  /* Panel was just untouched. */
    {
      if (valid)  /* Touch coordinates were valid. */
        {
          data.point[0].flags  = TOUCH_UP | TOUCH_ID_VALID | TOUCH_POS_VALID;
        }
      else  /* Touch coordinates were invalid. */
        {
          data.point[0].flags  = TOUCH_UP | TOUCH_ID_VALID;
        }
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
  iinfo("\n"); ////
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

  /* Read the touch data over I2C. */

  outlen = sizeof(struct touch_sample_s);
  if (buflen >= outlen)
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
  iinfo("\n"); ////
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
  DEBUGASSERT(use_count < UINT8_MAX && use_count > priv->cref);

  priv->cref = use_count;
  ret = 0;

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
  iinfo("\n"); ////
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

      bl602_irq_enable(false);

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
  iinfo("\n"); ////
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
  iinfo("\n"); ////
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

/****************************************************************************
 * Name: cst816s_isr_handler
 *
 * Description:
 *   Handle GPIO Interrupt triggered by touch.
 *
 ****************************************************************************/

static int cst816s_isr_handler(int _irq, FAR void *_context, FAR void *arg)
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
  iinfo("\n"); ////
  struct cst816s_dev_s *priv;
  int ret = 0;

  /* Allocate device private structure. */

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

  /* Prepare interrupt line and handler. */

  ret = bl602_irq_attach(BOARD_TOUCH_INT, cst816s_isr_handler, priv);
  if (ret < 0)
    {
      kmm_free(priv);
      ierr("Attach interrupt failed\n");
      return ret;
    }

  ret = bl602_irq_enable(false);
  if (ret < 0)
    {
      kmm_free(priv);
      ierr("Disable interrupt failed\n");
      return ret;
    }

  iinfo("Driver registered\n");

//  Uncomment this to test interrupts (tap the screen)
#define TEST_CST816S_INTERRUPT
#ifdef TEST_CST816S_INTERRUPT
#warning Testing CST816S interrupt
  bl602_irq_enable(true);
#endif /* TEST_CST816S_INTERRUPT */

  return 0;
}

/****************************************************************************
 * BL602 GPIO Interrupt. TODO: Move this to BL602 GPIO Expander
 * https://github.com/lupyuen/bl602_expander
 ****************************************************************************/

#include <nuttx/ioexpander/gpio.h>
#include <arch/board/board.h>
#include "../arch/risc-v/src/common/riscv_internal.h"
#include "../arch/risc-v/src/bl602/bl602_gpio.h"

static int bl602_expander_interrupt(int irq, void *context, void *arg);
static void bl602_expander_intmask(uint8_t gpio_pin, int intmask);
static void bl602_expander_set_intmod(uint8_t gpio_pin, uint8_t int_ctlmod, uint8_t int_trgmod);
static int bl602_expander_get_intstatus(uint8_t gpio_pin);
static void bl602_expander_intclear(uint8_t gpio_pin, uint8_t int_clear);

static gpio_pinset_t bl602_expander_pinset = 0;
static FAR isr_handler *bl602_expander_callback = NULL;
static FAR void *bl602_expander_arg = NULL;

//  struct ioexpander_dev_s;
//  typedef CODE int (*ioe_callback_t)(FAR struct ioexpander_dev_s *dev, ioe_pinset_t pinset, FAR void *arg);

/****************************************************************************
 * Name: bl602_irq_attach
 *
 * Description:
 *   Attach Interrupt Handler to GPIO Interrupt for Touch Controller.
 *   Based on https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L477-L505 
 *
 ****************************************************************************/

static int bl602_irq_attach(gpio_pinset_t pinset, FAR isr_handler *callback, FAR void *arg)
{
  int ret = 0;
  uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  FAR struct bl602_gpint_dev_s *dev = NULL;  //  TODO

  DEBUGASSERT(callback != NULL);

  /* Configure the pin that will be used as interrupt input */

  #warning Check GLB_GPIO_INT_TRIG_NEG_PULSE  ////  TODO
  bl602_expander_set_intmod(gpio_pin, 1, GLB_GPIO_INT_TRIG_NEG_PULSE);
  ret = bl602_configgpio(pinset);
  if (ret < 0)
    {
      gpioerr("Failed to configure GPIO pin %d\n", gpio_pin);
      return ret;
    }

  /* Make sure the interrupt is disabled */

  bl602_expander_pinset = pinset;
  bl602_expander_callback = callback;
  bl602_expander_arg = arg;
  bl602_expander_intmask(gpio_pin, 1);

  irq_attach(BL602_IRQ_GPIO_INT0, bl602_expander_interrupt, dev);
  bl602_expander_intmask(gpio_pin, 0);

  gpioinfo("Attach %p\n", callback);

  return 0;
}

/****************************************************************************
 * Name: bl602_irq_enable
 *
 * Description:
 *   Enable or disable GPIO Interrupt for Touch Controller.
 *   Based on https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L507-L535
 *
 ****************************************************************************/

static int bl602_irq_enable(bool enable)
{
  if (enable)
    {
      if (bl602_expander_callback != NULL)
        {
          gpioinfo("Enable interrupt\n");
          up_enable_irq(BL602_IRQ_GPIO_INT0);
        }
      else
        {
          gpiowarn("No callback attached\n");
        }
    }
  else
    {
      gpioinfo("Disable interrupt\n");
      up_disable_irq(BL602_IRQ_GPIO_INT0);
    }

  return 0;
}

/****************************************************************************
 * Name: bl602_expander_interrupt
 *
 * Description:
 *   Handle GPIO Interrupt. Based on
 *   https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L256-L304
 *
 ****************************************************************************/

static int bl602_expander_interrupt(int irq, void *context, void *arg)
{
  ////TODO: FAR struct bl602_gpint_dev_s *dev = (FAR struct bl602_gpint_dev_s *)arg;
  uint32_t time_out = 0;
  uint8_t gpio_pin;

  ////TODO: DEBUGASSERT(dev != NULL);
  DEBUGASSERT(bl602_expander_callback != NULL);
  gpioinfo("Interrupt! callback=%p, arg=%p\n", bl602_expander_callback, bl602_expander_arg);

  gpio_pin = (bl602_expander_pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;

  if (1 == bl602_expander_get_intstatus(gpio_pin))
    {
      bl602_expander_intclear(gpio_pin, 1);

      /* timeout check */

      time_out = 32;
      do
        {
          time_out--;
        }
      while ((1 == bl602_expander_get_intstatus(gpio_pin)) && time_out);
      if (!time_out)
        {
          gpiowarn("WARNING: Clear GPIO interrupt status fail.\n");
        }

      /* if time_out==0, GPIO interrupt status not cleared */

      bl602_expander_intclear(gpio_pin, 0);
    }

  gpioinfo("Call callback=%p, arg=%p\n", bl602_expander_callback, bl602_expander_arg);
  bl602_expander_callback(irq, context, bl602_expander_arg);
  ////TODO Previously: bl602_expander_callback(&bl602xgpint->bl602gpio.gpio, gpio_pin);

  return OK;
}

/****************************************************************************
 * Name: bl602_expander_intmask
 *
 * Description:
 *   Set Interrupt Mask for a GPIO Pin. Based on
 *   https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L143-L169
 *
 ****************************************************************************/

static void bl602_expander_intmask(uint8_t gpio_pin, int intmask)
{
  uint32_t tmp_val;

  if (gpio_pin < GPIO_PIN28)
    {
      tmp_val = getreg32(BL602_GPIO_INT_MASK1);
      if (intmask == 1)
        {
          tmp_val |= (1 << gpio_pin);
        }
      else
        {
          tmp_val &= ~(1 << gpio_pin);
        }

      putreg32(tmp_val, BL602_GPIO_INT_MASK1);
    }
  else
    {
      gpioerr("Invalid pin %d\n", gpio_pin);
      DEBUGPANIC();
    }
}

/****************************************************************************
 * Name: bl602_expander_set_intmod
 *
 * Description:
 *   Set GPIO Interrupt Mode. Based on
 *   https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L171-L212
 *
 ****************************************************************************/

static void bl602_expander_set_intmod(uint8_t gpio_pin,
              uint8_t int_ctlmod, uint8_t int_trgmod)
{
  gpioinfo("gpio_pin=%d, int_ctlmod=%d, int_trgmod=%d\n", gpio_pin, int_ctlmod, int_trgmod); ////
  uint32_t tmp_val;

  if (gpio_pin < GPIO_PIN10)
    {
      /* GPIO0 ~ GPIO9 */

      tmp_val = gpio_pin;
      modifyreg32(BL602_GPIO_INT_MODE_SET1,
                  0x7 << (3 * tmp_val),
                  ((int_ctlmod << 2) | int_trgmod) << (3 * tmp_val));
    }
  else if (gpio_pin < GPIO_PIN20)
    {
      /* GPIO10 ~ GPIO19 */

      tmp_val = gpio_pin - GPIO_PIN10;
      modifyreg32(BL602_GPIO_INT_MODE_SET2,
                  0x7 << (3 * tmp_val),
                  ((int_ctlmod << 2) | int_trgmod) << (3 * tmp_val));
    }
  else if (gpio_pin < GPIO_PIN28)
    {
      /* GPIO20 ~ GPIO27 */

      tmp_val = gpio_pin - GPIO_PIN20;
      modifyreg32(BL602_GPIO_INT_MODE_SET3,
                  0x7 << (3 * tmp_val),
                  ((int_ctlmod << 2) | int_trgmod) << (3 * tmp_val));
    }
  else
    {
      gpioerr("Invalid pin %d\n", gpio_pin);
      DEBUGPANIC();
    }
}

/****************************************************************************
 * Name: bl602_expander_get_intstatus
 *
 * Description:
 *   Get GPIO Interrupt Status. Based on
 *   https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L214-L234
 *
 ****************************************************************************/

static int bl602_expander_get_intstatus(uint8_t gpio_pin)
{
  uint32_t tmp_val = 0;

  if (gpio_pin < GPIO_PIN28)
    {
      /* GPIO0 ~ GPIO27 */

      tmp_val = getreg32(BL602_GPIO_INT_STAT1);
    }
  else
    {
      gpioerr("Invalid pin %d\n", gpio_pin);
      DEBUGPANIC();
    }

  return (tmp_val & (1 << gpio_pin)) ? 1 : 0;
}

/****************************************************************************
 * Name: bl602_expander_intclear
 *
 * Description:
 *   Clear GPIO Interrupt. Based on
 *   https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L236-L254
 *
 ****************************************************************************/

static void bl602_expander_intclear(uint8_t gpio_pin, uint8_t int_clear)
{
  if (gpio_pin < GPIO_PIN28)
    {
      /* GPIO0 ~ GPIO27 */

      modifyreg32(BL602_GPIO_INT_CLR1,
                  int_clear ? 0 : (1 << gpio_pin),
                  int_clear ? (1 << gpio_pin) : 0);
    }
  else
    {
      gpioerr("Invalid pin %d\n", gpio_pin);
      DEBUGPANIC();
    }
}
