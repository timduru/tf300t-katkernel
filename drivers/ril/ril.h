#ifndef RIL_H
#define RIL_H

#include "gpio-names.h"

/* DEBUG */
#define RIL_DEBUG 1

#if RIL_DEBUG
#define RIL_INFO(format, arg...) \
	printk(KERN_INFO "RIL: [%s] " format , __FUNCTION__ , ## arg)
#else
#define RIL_INFO(format, arg...)
#endif

#define RIL_ERR(format, arg...) \
	printk(KERN_ERR "RIL: [%s] " format , __FUNCTION__ , ## arg)

/* TF300TG and TF300TL common GPIOs */
#define MOD_VBUS_ON     TEGRA_GPIO_PD2
#define USB_SW_SEL      TEGRA_GPIO_PP1
#define SAR_DET_3G      TEGRA_GPIO_PR3
#define SIM_CARD_DET    TEGRA_GPIO_PW3

/* TF300TL GPIOs */
#define MOD_POWER_KEY   TEGRA_GPIO_PV6
#define MOD_VBAT_ON     TEGRA_GPIO_PC6
#define AP_TO_MOD_RST   TEGRA_GPIO_PN0
#define MOD_HANG        TEGRA_GPIO_PN2
#define MOD_SUS_REQ     TEGRA_GPIO_PX7
#define AP_WAKE_IND     TEGRA_GPIO_PEE0 //AP Wake MOD Ack
#define AP_WAKE_MOD     TEGRA_GPIO_PEE1
#define MOD_WAKE_AP     TEGRA_GPIO_PU5
#define MOD_WAKE_IND    TEGRA_GPIO_PEE2 //MOD Wake AP Ack
#define DL_MODE         TEGRA_GPIO_PN3
#define DL_COMPLETE     TEGRA_GPIO_PO6

extern int project_id;

#endif
