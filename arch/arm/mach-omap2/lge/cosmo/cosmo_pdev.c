/*
 * Platform Devices initializing.
 *
 * Copyright (C) 2010,2011,2012 LG Electronics, Inc.
 *
 * Author: Seungho Park <seungho1.park@lge.com>
 *         Do-Yeob Kim <doyeob.kim@lge.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/input.h>
#include <linux/bu52031nvx.h>
#include <plat/omap-serial.h>
#include <plat/gpio.h>
#include <lge/common.h>
#include <lge/board.h>
#include <lge/board_rev.h>
#include <linux/lge/pwm-vibrator.h>
#include <linux/lge/leds_keypad.h>
#include <linux/lge/lge_input.h>
//                                          
//20110505, ravi.holal@teleca.com, RIL RECOVERY
//Description: Modem watcher functionality header file
#include <linux/lge/mdm_watcher.h>
//                                        
#include <linux/switch_dp3t.h>
#include <linux/switch_usif.h>
//                                                                                 
#if defined(CONFIG_GPS)
#include <linux/lge/gps_gpio.h>
#endif /* CONFIG_GPS */
//                                                                                 
//                                               
#include <linux/lbee9qmb-rfkill.h>
//---
#if CONFIG_MACH_LGE_COSMO
#include <plat/keypad.h>
#endif

#if CONFIG_MACH_LGE_COSMO //nthyunjin.yang 120412
#include <linux/fuel_gauge_max17043.h>
#endif

// TIK - start
#include <linux/skbuff.h>
#include <linux/ti_wilink_st.h>
#define WILINK_UART_DEV_NAME    "/dev/ttyO1"
// TIK - end

//                                                              
#define BT_UART_DEV_NAME "/dev/ttyO1"

// TIK - start
/* TODO: handle suspend/resume here.
 * Upon every suspend, make sure the wilink chip is
 * capable enough to wake-up the OMAP host.
 */

//                                                     
static struct wake_lock bt_host_wake_lock; 
// +e LGBT_COMMON_HOST_WAKE

static int plat_wlink_kim_suspend(struct platform_device *pdev, pm_message_t
                state)
{
	printk("%s\n", __func__);
        return 0;
}

static int plat_wlink_kim_resume(struct platform_device *pdev)
{
//                                                     
	printk("%s\n", __func__);
	wake_lock_timeout(&bt_host_wake_lock, 3*HZ);
// +e LGBT_COMMON_HOST_WAKE
        return 0;
}
// TIK - end
static bool uart_req;

// TIK - start
static struct wake_lock st_wk_lock; 

/* Call the uart disable of serial driver */
static int plat_uart_disable(void)
{
        int port_id = 0;
        int err = 0;
	  printk("%s\n", __func__);
        if (uart_req) {
		    printk("%s: omap_serial_ext_uart_disable\n", __func__);
                sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
                err = omap_serial_ext_uart_disable(port_id);
                if (!err)
                        uart_req = false;
        }
        wake_unlock(&st_wk_lock);
        return err;
}

/* Call the uart enable of serial driver */
static int plat_uart_enable(void)
{
        int port_id = 0;
        int err = 0;
	  printk("%s\n", __func__);
        if (!uart_req) {
		    printk("%s: omap_serial_ext_uart_enable\n", __func__);
                sscanf(WILINK_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
                err = omap_serial_ext_uart_enable(port_id);
                if (!err)
                        uart_req = true;
        }
        wake_lock(&st_wk_lock);
        return err;
}

/* wl128x BT, FM, GPS connectivity chip */
static struct ti_st_plat_data wilink_pdata = {
        .nshutdown_gpio = 166,
        .dev_name = WILINK_UART_DEV_NAME,
        .flow_cntrl = 1,
        .baud_rate = 3686400,
        .suspend = plat_wlink_kim_suspend,
        .resume = plat_wlink_kim_resume,
        .chip_asleep = plat_uart_disable,
        .chip_awake  = plat_uart_enable,
        .chip_enable = plat_uart_enable,
        .chip_disable = plat_uart_disable,
};

static struct platform_device wl128x_device = {
        .name           = "kim",
        .id             = -1,
        .dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
        .name = "btwilink",
        .id = -1,
};
// TIK - end
/* Call the uart disable of serial driver */
static int plat_bt_uart_disable(void)
{
        int port_id = 0;
        int err = 0;
        if (uart_req) {
                sscanf(BT_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
                err = omap_serial_ext_uart_disable(port_id);
                if (!err)
                        uart_req = false;
        }
        return err;
}

/* Call the uart enable of serial driver */
static int plat_bt_uart_enable(void)
{
        int port_id = 0;
        int err = 0;
        if (!uart_req) {
                sscanf(BT_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
                err = omap_serial_ext_uart_enable(port_id);
                if (!err)
                        uart_req = true;
        }
        return err;
}
//                                                              


//                                                                                                                                  
/*
                                                          
                             
                          
                             
       
                            
                                    
       
                 
             
                                                                
                                           
                                             
                                                                

  

                                                
                           
         
                                      
   
  
*/

#if defined(CONFIG_INPUT_HALLEFFECT_BU52031NVX)
static struct bu52031nvx_platform_data bu52031nvx_platform_data = {
	.docked_gpio = GPIO_HALL_INT,
	.is_desk = 1,
};

static struct platform_device omap3430_hall_effect_dock = {
	.name	= BU52031NVX_MODULE_NAME,
	.id	= -1,
	.dev	= {
		.platform_data  = &bu52031nvx_platform_data,
	},
};
#endif



//---

//                                             
//ADD: 0018892 [B2][BT] FactoryTest AT command
//                                                                            
struct platform_device bd_address_device = {
	.name = "bd_address",
	.id = -1,
};
//                                                                          
//                                           

//                                   
#if defined(CONFIG_I2C_GPIO)
#include <linux/i2c-gpio.h>
#define GPIO_MHL_I2C_SCL	8
#if CONFIG_MACH_LGE_CX2_SU870 //nthyunjin.yang 120421
#define GPIO_MHL_I2C_SDA	182
#else
#define GPIO_MHL_I2C_SDA	31
#endif

enum {
	MHL_GPIO_I2C = 0,
	MAX_GPIO_I2C,
};
#define I2C_ID(x) (x + 5)

static struct i2c_gpio_platform_data i2c_gpio_data[] = {
    {
        .sda_pin = GPIO_MHL_I2C_SDA,
        .scl_pin = GPIO_MHL_I2C_SCL, 
        .sda_is_open_drain = 0,
        .scl_is_open_drain = 0,
        .udelay = 2, //10,
    },
};

static struct platform_device i2c_gpio_device[] = {
	{
	    .name = "i2c-gpio",
	    .id = I2C_ID(MHL_GPIO_I2C),
	    .dev.platform_data = &i2c_gpio_data[MHL_GPIO_I2C],
	},
};

struct i2c_board_info i2c_gpio_info_mhl[] = {
#if defined(CONFIG_MHL_TX_SII9244) || defined(CONFIG_MHL_TX_SII9244_LEGACY)
	{
		I2C_BOARD_INFO("sii9244_mhl_tx", 0x72>>1),
		.irq = OMAP_GPIO_IRQ(GPIO_MHL_INT),
		.platform_data = &sii9244_pdata,
	},
	{
		I2C_BOARD_INFO("sii9244_tpi", 0x7A>>1),
		.platform_data = &sii9244_pdata,
	},
	{
		I2C_BOARD_INFO("sii9244_hdmi_rx", 0x92>>1),
		.platform_data = &sii9244_pdata,
	},
	{
		I2C_BOARD_INFO("sii9244_cbus", 0xC8>>1),
		.platform_data = &sii9244_pdata,
	},
#endif	
};

static void __init lge_add_i2c_gpio_device(void)
{
	int i, ret;

	for (i = 0; i < MAX_GPIO_I2C; i++) {
		platform_device_register(&i2c_gpio_device[i]);
	}
	printk( "[MHL]i2c_register_board_info\n");
	ret = i2c_register_board_info(I2C_ID(MHL_GPIO_I2C), i2c_gpio_info_mhl,
                 ARRAY_SIZE(i2c_gpio_info_mhl));
	printk( "[MHL]i2c_register_board_info ret:%d\n", ret);
}
#endif
//                                              


//                                                                                 
#if defined(CONFIG_GPS)
//                                                                       
#define GPS_UART_DEV_NAME "/dev/ttyO2"

static bool gps_uart_req;
/* Call the uart disable of serial driver */
static int plat_gps_uart_disable(void)
{
        int port_id = 0;
        int err = 0;
        if (gps_uart_req) {
                sscanf(GPS_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
                err = omap_serial_ext_uart_disable(port_id);
                if (!err)
                        gps_uart_req = false;
        }
        return err;
}

/* Call the uart enable of serial driver */
static int plat_gps_uart_enable(void)
{
        int port_id = 0;
        int err = 0;
        if (!gps_uart_req) {
                sscanf(GPS_UART_DEV_NAME, "/dev/ttyO%d", &port_id);
                err = omap_serial_ext_uart_enable(port_id);
                if (!err)
                        gps_uart_req = true;
        }
        return err;
}
//                                                                       

static struct gps_gpio_platform_data gps_pdata = {
        .pwron   = GPS_PWR_ON_GPIO,
        .reset_n = GPS_RESET_N_GPIO,
#if defined(CONFIG_P940_GPS_LNA_SD_USE)
        .lna_sd  = GPS_LNA_SD_GPIO,
#endif
//                                                                       
        .uart_dev_name = GPS_UART_DEV_NAME,
        .uart_enable = plat_gps_uart_enable,
        .uart_disable = plat_gps_uart_disable,
//                                                                       

};
static struct platform_device gps_gpio =
{
        .name = "gps_gpio",
        .id       = -1,
        .dev = {
                .platform_data = &gps_pdata,
        }
};
#endif /* CONFIG_GPS */
//                                                                                 

//                                                                      
/*                                             */

//                                                                             
extern int vibrator_power_control(bool on);
/*                                                    */
static struct pwm_vibrator_platform_data vib_data = {
	.freq			= 227,
	.gpio_enable		= 25,
	.power			= vibrator_power_control,
	.port			= 8,
};
//                                            

static struct platform_device vib = {
	.name = "tspdrv",
	.id	  = -1,
	.dev  = {
		.platform_data = &vib_data,
	},
};

/*                                                                           */
static struct leds_keypad_platform_data keypad_led_pdata = {
	.name          = "button-backlight",
	.use_hold_key  = 0,
	.hold_key_gpio   = 81,
	.keypad_gpio	= 82,
};

static struct platform_device keypad_led = {
	.name = "keypad_led",
	.id	  = -1,
	.dev = {
		.platform_data = &keypad_led_pdata,
	},
};

void keyboard_mux_init(void)
{
	return;
}

/*                                                                                  */
#if defined(CONFIG_LGE_MTC_ETA)
struct platform_device mtc_eta_log_device = {
	.name = "lge_mtc_eta_logger",
};
#endif
/*                                                 */

static struct platform_device charger_device= {
	.name = "cosmo_charger",
	.id	  = -1,
};

//                                          
//20110505, ravi.holal@teleca.com, RIL RECOVERY
//Description: Describes the GPIO configuration for Modem reset feature
//                                                     
//MOD : RIP-11133 : added CP coredump feature
static struct mdm_watcher_event mdm_watcher_event_data[] __initdata  = {
	{
		.type = MDM_HALT,
#if defined(CONFIG_LGE_SPI_SLAVE)
		.gpio_irq = 56, /* CP_CRASH_INT for mdm*/
		.irqf_flags = IRQF_TRIGGER_FALLING | IRQF_DISABLED,
#else
		.gpio_irq = 26, /* CP_CRASH_INT for ifx*/
		.gpio_irq_srdy = 119, /* IPC_SRDY for ifx*/
		.gpio_irq_mrdy = 120, /* IPC_MRDY for ifx*/
		
		.irqf_flags = IRQF_TRIGGER_RISING | IRQF_DISABLED,
#endif

		.msecs_delay = 5,
		.key_code = 194,
		.key_code_srdy = 196,
		.key_code_mrdy = 198,
	},
//                                                   
#if 0 /* TODO: there is no gpio pin for auto_shutdown */
	{
		.type = MDM_AUTO_SHUTDOWN,
		.gpio_irq = -1, /* ???  */
		.irqf_flags = IRQF_TRIGGER_FALLING,
		.msecs_delay = 5,
		.key_code = 195,
	}
#endif
};
static struct mdm_watcher_platform_data cosmo_mdm_watcher_pdata __refdata = {
	.event = mdm_watcher_event_data,
	.len = ARRAY_SIZE(mdm_watcher_event_data),
};

static struct platform_device cosmo_mdm_watcher_device __refdata = {
	.name = MDM_WATCHER_NAME,
	.id = -1,
	.dev  = {
		.platform_data = &cosmo_mdm_watcher_pdata,
	},
};
//                                        

#if defined(CONFIG_LGE_HANDLE_PANIC)
static struct resource crash_log_resource[] = {
	{
		.name = "crash_log",
		.flags = IORESOURCE_MEM,
		.start = LGE_CRASH_LOG_START,
		.end = LGE_CRASH_LOG_START + LGE_CRASH_LOG_SIZE - 1,
	}
};

static struct platform_device panic_handler_device = {
	.name = "panic-handler",
	.id = -1,
	.num_resources = ARRAY_SIZE(crash_log_resource),
	.resource = crash_log_resource,
	.dev    = {
		.platform_data = NULL,
	}
};
#endif

/*                                                      */
#ifdef CONFIG_LGE_GPIO_CONTROL
static struct platform_device gpio_control = {
	.name           = "gpio_control",
	.id             = -1,
};
#endif
/*                                                    */

struct dp3t_switch_platform_data dp3t_pdata = {
	.ctrl_gpio1 = GPIO_DP3T_IN_1,
	.ctrl_gpio2 = GPIO_DP3T_IN_2,
	.ctrl_ifx_vbus_gpio = GPIO_IFX_USB_VBUS_EN,
};

static struct platform_device dp3t_dev = {
	.name = "switch-dp3t",
	.id	  = -1,
	.dev	= {	
		.platform_data = &dp3t_pdata,
	},
};

struct usif_switch_platform_data usif_pdata = {
	.ctrl_gpio = GPIO_USIF_IN_1,
};

static struct platform_device usif_dev = {
	.name = "switch-usif",
	.id	  = -1,
	.dev	= {	
		.platform_data = &usif_pdata,
	},
};

/* gkpd */
static struct key_table gkpd_keys[] = {
	{'D', KEY_VOLUMEDOWN},
	{'U', KEY_VOLUMEUP},
#if defined(CONFIG_MACH_LGE_COSMO)
	{'F', KEY_3D},
#else
	{'F', KEY_CAPTURE},
#endif //##
	{'H', KEY_HOOK},
	{'I', KEY_TESTMODE_UNLOCK},
	{0, 0}
};

static struct gkpd_platform_data gkpd_pdata = {
	.keys = gkpd_keys,
	.size = ARRAY_SIZE(gkpd_keys),
};

static struct platform_device gkpd_dev = {
	.name = "lge-gkpd",
	.id   = -1,
	.dev  = {
		.platform_data = &gkpd_pdata,
	},
};

#ifdef CONFIG_MACH_LGE_COSMO //nthyunjin.yang 120412
struct max17043_platform_data max17043_pdata = {
	.slave_addr  = FUEL_GAUGE_MAX17043_SLAVE_ID,
	.gpio_alert	= FUEL_GAUGE_MAX17043_GPIO_ALERT,	/* active low */
	.gpio_scl	= FUEL_GAUGE_MAX17043_GPIO_SCL,
	.gpio_sda	= FUEL_GAUGE_MAX17043_GPIO_SDA,
	.udelay		= FUEL_GAUGE_MAX17043_UDELAY,
	.timeout	= FUEL_GAUGE_MAX17043_TIMEOUT,
	.rcomp 		= RCOMP_BL48LN,
};

static struct platform_device cosmo_fuel_gauge_device= {
	.name		= "max17043_gpio",
	.id		= -1,
	.dev.platform_data = &max17043_pdata,
};
#endif

static struct platform_device *pdevs[] __initdata = {
//                                                                                 
#if defined(CONFIG_GPS)
        &gps_gpio,
#endif /* CONFIG_GPS */
//                                                                                 

#if defined(CONFIG_LGE_MTC_ETA)
	&mtc_eta_log_device,
#endif
	&vib,
	&keypad_led,
	&charger_device,
#ifdef CONFIG_MACH_LGE_COSMO //nthyunjin.yang 120420
	&cosmo_fuel_gauge_device,
#endif
//                                               
//	&bcm4330_device,
//                                             
//ADD: 0018892 [B2][BT] FactoryTest AT command
	//                                                                            
	&bd_address_device,
	//                                                                          
//                                           
// TIK - start
	&wl128x_device,
	&btwilink_device,
// TIK - end

//                                          
//20110505, ravi.holal@teleca.com, RIL RECOVERY
//Description: Adding the cosmo_mdm_watcher_device structure to platform	
	&cosmo_mdm_watcher_device,
//                                         
#ifdef CONFIG_LGE_HANDLE_PANIC
	&panic_handler_device,
#endif
/*                                                      */
#ifdef CONFIG_LGE_GPIO_CONTROL 
	&gpio_control,
#endif
/*                                                    */
	&dp3t_dev,
	&usif_dev,
	&gkpd_dev,
};

/*                                                      */
static char vib_name_b[] = VIB_PWM_NAME;

int __init cosmo_pdevs_init(void)
{
//	lge_add_i2c_gpio_device();

	keypad_led_pdata.keypad_gpio = 82;
// TIK - start
      wake_lock_init(&st_wk_lock, WAKE_LOCK_SUSPEND, "st_wake_lock");
	wake_lock_init(&bt_host_wake_lock, WAKE_LOCK_SUSPEND, "bt_host_wake_lock");
// TIK - end
	return lge_set_pdevs(pdevs, ARRAY_SIZE(pdevs));
}

lge_machine_initcall(cosmo_pdevs_init);
