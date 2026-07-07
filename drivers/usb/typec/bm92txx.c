// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ROHM BM92TxX USB Type-C / USB Power Delivery controller
 *
 * Copyright (c) 2020-2023 CTCaer <ctcaer@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/build_bug.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/extcon-provider.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/usb/role.h>

#define BM92T_VDM_WAIT_RETRIES 400
#define BM92T_USBHUB_RETRIES   10

/* Registers */
#define ALERT_STATUS_REG    0x02
#define STATUS1_REG         0x03
#define STATUS2_REG         0x04
#define COMMAND_REG         0x05 /* Send special command */
#define CONFIG1_REG         0x06 /* Controller Configuration 1 */
#define DEV_CAPS_REG        0x07
#define READ_PDOS_SRC_REG   0x08 /* Data size: 28 */
#define CONFIG2_REG         0x17 /* Controller Configuration 2 */
#define DP_STATUS_REG       0x18
#define DP_ALERT_EN_REG     0x19
#define VENDOR_CONFIG_REG   0x1A /* Vendor Configuration 1 */
#define UNKOWN_1C_REG       0x1C /* HOS reads it. 2 bytes. DP sth */
#define UNKOWN_1D_REG       0x1D /* HOS reads it. 2 bytes. DP sth */
#define AUTO_NGT_FIXED_REG  0x20 /* Data size: 4 */
#define AUTO_NGT_BATT_REG   0x23 /* Data size: 4 */
#define SYS_CONFIG1_REG     0x26 /* System Configuration 1 */
#define SYS_CONFIG2_REG     0x27 /* System Configuration 2 */
#define CURRENT_PDO_REG     0x28 /* Data size: 4 */
#define CURRENT_RDO_REG     0x2B /* Data size: 4 */
#define ALERT_ENABLE_REG    0x2E
#define SYS_CONFIG3_REG     0x2F /* System Configuration 3 */
#define SET_RDO_REG         0x30 /* Data size: 4 */
#define PDOS_SNK_CONS_REG   0x33 /* PDO Sink Consumer. Data size: 16 */
#define PDOS_SRC_PROV_REG   0x3C /* PDO Source Provider. Data size: 28 */
#define FW_TYPE_REG         0x4B
#define FW_REVISION_REG     0x4C
#define MAN_ID_REG          0x4D
#define DEV_ID_REG          0x4E
#define REV_ID_REG          0x4F
#define INCOMING_VDM_REG    0x50 /* Max data size: 28 */
#define OUTGOING_VDM_REG    0x60 /* Max data size: 28 */

/* ALERT_STATUS_REG */
#define ALERT_SNK_FAULT     BIT(0)
#define ALERT_SRC_FAULT     BIT(1)
#define ALERT_CMD_DONE      BIT(2)
#define ALERT_PLUGPULL      BIT(3)
#define ALERT_DP_EVENT      BIT(6)
#define ALERT_DR_SWAP       BIT(10)
#define ALERT_VDM_RECEIVED  BIT(11)
#define ALERT_CONTRACT      BIT(12)
#define ALERT_SRC_PLUGIN    BIT(13)
#define ALERT_PDO           BIT(14)

/* STATUS1_REG */
#define STATUS1_FAULT_MASK    (3 << 0)
#define STATUS1_SPDSRC2       BIT(3) /* VBUS2 enabled */
#define STATUS1_LASTCMD_SHIFT 4
#define STATUS1_LASTCMD_MASK  (7 << STATUS1_LASTCMD_SHIFT)
#define STATUS1_INSERT        BIT(7)  /* Cable inserted */
#define STATUS1_DR_SHIFT      8
#define STATUS1_DR_MASK       (3 << STATUS1_DR_SHIFT)
#define STATUS1_VSAFE         BIT(10) /* 0: No power, 1: VSAFE 5V or PDO */
#define STATUS1_CSIDE         BIT(11) /* Type-C Plug Side. 0: CC1 Side Valid, 1: CC2 Side Valid */
#define STATUS1_SRC_MODE      BIT(12) /* 0: Sink Mode, 1: Source mode (OTG) */
#define STATUS1_CMD_BUSY      BIT(13) /* Command in progress */
#define STATUS1_SPDSNK        BIT(14) /* Sink mode */
#define STATUS1_SPDSRC1       BIT(15) /* VBUS enabled */

#define LASTCMD_COMPLETE   0
#define LASTCMD_ABORTED    2
#define LASTCMD_INVALID    4
#define LASTCMD_REJECTED   6
#define LASTCMD_TERMINATED 7

#define DATA_ROLE_NONE  0
#define DATA_ROLE_UFP   1
#define DATA_ROLE_DFP   2
#define DATA_ROLE_ACC   3

/* STATUS2_REG */
#define STATUS2_PDOI_MASK    BIT(3)
#define STATUS2_VCONN_ON     BIT(9)
#define STATUS2_ACC_SHIFT    10
#define STATUS2_ACC_MASK     (3 << STATUS2_ACC_SHIFT) /* Accesory mode */
#define STATUS2_EM_CABLE     BIT(12) /* Electronically marked cable. Safe for 1.3A */
#define STATUS2_OTG_INSERT   BIT(13)

#define PDOI_SRC_OR_NO  0
#define PDOI_SNK        1

#define ACC_DISABLED    0
#define ACC_AUDIO       1
#define ACC_DEBUG       2
#define ACC_VCONN       3

/* DP_STATUS_REG */
#define DP_STATUS_PIN_CFG_DONE BIT(1) /* Pin configured or sth */
/* TV/Monitor connected: link or HPD channel enabled */
#define DP_STATUS_SIGNAL_ON    BIT(7)
#define DP_STATUS_INSERT       BIT(14)
#define DP_STATUS_DP_EN        BIT(15)

/* CONFIG1_REG */
#define CONFIG1_AUTO_DR_SWAP          BIT(1)
#define CONFIG1_SLEEP_REQUEST         BIT(4)
#define CONFIG1_AUTONGTSNK_VAR_EN     BIT(5)
#define CONFIG1_AUTONGTSNK_FIXED_EN   BIT(6)
#define CONFIG1_AUTONGTSNK_EN         BIT(7)
#define CONFIG1_AUTONGTSNK_BATT_EN    BIT(8)
#define CONFIG1_VINOUT_DELAY_EN       BIT(9) /* VIN/VOUT turn on delay enable */
#define CONFIG1_VINOUT_TIME_ON_SHIFT  10 /* VIN/VOUT turn on delay */
#define CONFIG1_VINOUT_TIME_ON_MASK   (3 << CONFIG1_VINOUT_TIME_ON_SHIFT)
#define CONFIG1_SPDSRC_SHIFT          14
#define CONFIG1_SPDSRC_MASK           (3 << CONFIG1_SPDSRC_SHIFT)

#define VINOUT_TIME_ON_1MS    0
#define VINOUT_TIME_ON_5MS    1
#define VINOUT_TIME_ON_10MS   2
#define VINOUT_TIME_ON_20MS   3

#define SPDSRC12_ON           0 /* SPDSRC 1/2 on */
#define SPDSRC2_ON            1
#define SPDSRC1_ON            2
#define SPDSRC12_OFF          3 /* SPDSRC 1/2 off */

/* CONFIG2_REG */
#define CONFIG2_PR_SWAP_MASK      (3 << 0)
#define CONFIG2_DR_SWAP_SHIFT     2
#define CONFIG2_DR_SWAP_MASK      (3 << CONFIG2_DR_SWAP_SHIFT)
#define CONFIG2_VSRC_SWAP         BIT(4) /* VCONN source swap. 0: Reject, 1: Accept */
#define CONFIG2_NO_USB_SUSPEND    BIT(5)
#define CONFIG2_EXT_POWERED       BIT(7)
#define CONFIG2_TYPEC_AMP_SHIFT   8
#define CONFIG2_TYPEC_AMP_MASK    (3 << CONFIG2_TYPEC_AMP_SHIFT)

#define PR_SWAP_ALWAYS_REJECT         0
#define PR_SWAP_ACCEPT_SNK_REJECT_SRC 1 /* Accept when power sink */
#define PR_SWAP_ACCEPT_SRC_REJECT_SNK 2 /* Accept when power source */
#define PR_SWAP_ALWAYS_ACCEPT         3

#define DR_SWAP_ALWAYS_REJECT         0
#define DR_SWAP_ACCEPT_UFP_REJECT_DFP 1 /* Accept when device */
#define DR_SWAP_ACCEPT_DFP_REJECT_UFP 2 /* Accept when host */
#define DR_SWAP_ALWAYS_ACCEPT         3

#define TYPEC_AMP_0_5A_5V   0
#define TYPEC_AMP_1_5A_5V   1
#define TYPEC_AMP_3_0A_5V   2

/* SYS_CONFIG1_REG */
#define SYS_CONFIG1_PLUG_MASK           (0xF << 0)
#define SYS_CONFIG1_USE_AUTONGT         BIT(6)
#define SYS_CONFIG1_PDO_SNK_CONS        BIT(8)
#define SYS_CONFIG1_PDO_SNK_CONS_SHIFT  9 /* Number of Sink PDOs */
#define SYS_CONFIG1_PDO_SNK_CONS_MASK   (7 << SYS_CONFIG1_PDO_SNK_CONS_SHIFT)
#define SYS_CONFIG1_PDO_SRC_PROV        BIT(12)
#define SYS_CONFIG1_DOUT4_SHIFT         13
#define SYS_CONFIG1_DOUT4_MASK          (3 << SYS_CONFIG1_DOUT4_SHIFT)
#define SYS_CONFIG1_WAKE_ON_INSERT      BIT(15)

#define PLUG_TYPE_C      9
#define PLUG_TYPE_C_3A   10
#define PLUG_TYPE_C_5A   11

#define DOUT4_PDO4       0
#define DOUT4_PDO5       1
#define DOUT4_PDO6       2
#define DOUT4_PDO7       3

/* SYS_CONFIG2_REG */
#define SYS_CONFIG2_NO_COMM_UFP          BIT(0) /* Force no USB comms Capable UFP */
#define SYS_CONFIG2_NO_COMM_DFP          BIT(1) /* Force no USB comms Capable DFP */
#define SYS_CONFIG2_NO_COMM_ON_NO_BATT   BIT(2) /* Force no USB comms on dead battery */
#define SYS_CONFIG2_AUTO_SPDSNK_EN       BIT(6) /* Enable SPDSNK without SYS_RDY */
#define SYS_CONFIG2_BST_EN               BIT(8)
#define SYS_CONFIG2_PDO_SRC_PROV_SHIFT   9 /* Number of Source provisioned PDOs */
#define SYS_CONFIG2_PDO_SRC_PROV_MASK    (7 << SYS_CONFIG2_PDO_SRC_PROV_SHIFT)

/* VENDOR_CONFIG_REG */
#define VENDOR_CONFIG_OCP_DISABLE  BIT(2) /* Disable Over-current protection */

/* DEV_CAPS_REG */
#define DEV_CAPS_ALERT_STS  BIT(0)
#define DEV_CAPS_ALERT_EN   BIT(1)
#define DEV_CAPS_VIN_EN     BIT(2)
#define DEV_CAPS_VOUT_EN0   BIT(3)
#define DEV_CAPS_SPDSRC2    BIT(4)
#define DEV_CAPS_SPDSRC1    BIT(5)
#define DEV_CAPS_SPRL       BIT(6)
#define DEV_CAPS_SPDSNK     BIT(7)
#define DEV_CAPS_OCP        BIT(8)  /* Over current protection */
#define DEV_CAPS_DP_SRC     BIT(9)  /* DisplayPort capable Source */
#define DEV_CAPS_DP_SNK     BIT(10) /* DisplayPort capable Sink */
#define DEV_CAPS_VOUT_EN1   BIT(11)

/* COMMAND_REG command list */
#define ABORT_LASTCMD_SENT_CMD    0x0101
#define PR_SWAP_CMD               0x0303 /* Power Role swap request */
#define PS_RDY_CMD                0x0505 /* Power supply ready */
#define GET_SRC_CAP_CMD           0x0606 /* Get Source capabilities */
#define SEND_RDO_CMD              0x0707
#define PD_HARD_RST_CMD           0x0808 /* Hard reset link */
#define STORE_SYSCFG_CMD          0x0909 /* Store system configuration */
#define UPDATE_PDO_SRC_PROV_CMD   0x0A0A /* Update PDO Source Provider */
#define GET_SNK_CAP_CMD           0x0B0B /* Get Sink capabilities */
#define STORE_CFG2_CMD            0x0C0C /* Store controller configuration 2 */
#define SYS_RESET_CMD             0x0D0D /* Full USB-PD IC reset */
#define RESET_PS_RDY_CMD          0x1010 /* Reset power supply ready */
#define SEND_VDM_CMD              0x1111 /* Send VMD SOP */
#define SEND_VDM_1_CMD            0x1212 /* Send VMD SOP'  EM cable near end */
#define SEND_VDM_2_CMD            0x1313 /* Send VMD SOP'' EM cable far end */
#define SEND_VDM_1_DBG_CMD        0x1414 /* Send VMD SOP'  debug */
#define SEND_VDM_2_DBG_CMD        0x1515 /* Send VMD SOP'' debug */
#define ACCEPT_VDM_CMD            0x1616 /* Receive VDM */
#define MODE_ENTERED_CMD          0x1717 /* Alt mode entered */
#define DR_SWAP_CMD               0x1818 /* Data Role swap request */
#define VC_SWAP_CMD               0x1919 /* VCONN swap request */
#define BIST_REQ_CARR_M2_CMD      0x2424 /* Request BIST carrier mode 2 */
#define BIST_TEST_DATA_CMD        0x2B2B /* Send BIST test data */
#define PD_SOFT_RST_CMD           0x2C2C /* Reset power and get new PDO/Contract */
#define BIST_CARR_M2_CONT_STR_CMD 0x2F2F /* Send BIST carrier mode 2 continuous string */
#define DP_ENTER_MODE_CMD         0x3131 /* Discover DP Alt mode */
#define DP_STOP_CMD               0x3232 /* Cancel DP Alt mode discovery */
#define START_HPD_CMD             0x3434 /* Start handling HPD */
/* Configure and enter the selected DP Alt mode, then start handling HPD */
#define DP_CFG_AND_START_HPD_CMD  0x3636
#define STOP_HPD_CMD              0x3939 /* Stop handling HPD */
#define STOP_HPD_EXIT_DP_CMD      0x3B3B /* Stop handling HPD and exit DP Alt mode */

/* General defines */
#define PDO_TYPE_FIXED  0
#define PDO_TYPE_BATT   1
#define PDO_TYPE_VAR    2

#define PDO_INFO_DR_DATA   BIT(5)
#define PDO_INFO_USB_COMM  BIT(6)
#define PDO_INFO_EXT_POWER BIT(7)
#define PDO_INFO_HP_CAP    BIT(8)
#define PDO_INFO_DR_POWER  BIT(9)

/* VDM/VDO */
#define VDM_CMD_RESERVED    0x00
#define VDM_CMD_DISC_ID     0x01
#define VDM_CMD_DISC_SVID   0x02
#define VDM_CMD_DISC_MODE   0x03
#define VDM_CMD_ENTER_MODE  0x04
#define VDM_CMD_EXIT_MODE   0x05
#define VDM_CMD_ATTENTION   0x06
#define VDM_CMD_DP_STATUS   0x10
#define VDM_CMD_DP_CONFIG   0x11

#define VDM_ACK   0x40
#define VDM_NAK   0x80
#define VDM_BUSY  0xC0
#define VDM_UNSTRUCTURED   0x00
#define VDM_STRUCTURED     0x80

/* VDM Discover ID */
#define VDO_ID_TYPE_NONE        0
#define VDO_ID_TYPE_PD_HUB      1
#define VDO_ID_TYPE_PD_PERIPH   2
#define VDO_ID_TYPE_PASS_CBL    3
#define VDO_ID_TYPE_ACTI_CBL    4
#define VDO_ID_TYPE_ALTERNATE   5

/* VDM Discover Mode Caps [From device (UFP_U) to host (DFP_U)] */
#define VDO_DP_UFP_D       BIT(0) /* DisplayPort Sink */
#define VDO_DP_DFP_D       BIT(1) /* DisplayPort Source */
#define VDO_DP_SUPPORT     BIT(2)
#define VDO_DP_RECEPTACLE  BIT(6)

/* VDM DP Configuration [From host (DFP_U) to device (UFP_U)] */
#define VDO_DP_U_DFP_D     BIT(0) /* UFP_U as DisplayPort Source */
#define VDO_DP_U_UFP_D     BIT(1) /* UFP_U as DisplayPort Sink */
#define VDO_DP_SUPPORT     BIT(2)
#define VDO_DP_RECEPTACLE  BIT(6)

/* VDM Mode Caps and DP Configuration pins */
#define VDO_DP_PIN_A   BIT(0)
#define VDO_DP_PIN_B   BIT(1)
#define VDO_DP_PIN_C   BIT(2)
#define VDO_DP_PIN_D   BIT(3)
#define VDO_DP_PIN_E   BIT(4)
#define VDO_DP_PIN_F   BIT(5)

/* Known VID/SVID */
#define VID_NINTENDO      0x057E
#define PID_NIN_DOCK      0x2003
#define PID_NIN_CHARGER   0x2004

#define SVID_NINTENDO     VID_NINTENDO
#define SVID_DP           0xFF01

/* Nintendo dock VDM Commands */
#define VDM_NCMD_LED_CONTROL         0x01 /* Reply size 12 */
#define VDM_NCMD_DEVICE_STATE        0x16 /* Reply size 12 */
#define VDM_NCMD_DP_SIGNAL_DISABLE   0x1C /* Reply size 8 */
#define VDM_NCMD_HUB_RESET           0x1E /* Reply size 8 */
#define VDM_NCMD_HUB_CONTROL         0x20 /* Reply size 8 */

/* Nintendo dock VDM Request Type */
#define VDM_ND_READ    0
#define VDM_ND_WRITE   1

/* Nintendo dock VDM Reply Status */
#define VDM_ND_BUSY    1

/* Nintendo dock VDM Request/Reply Source */
#define VDM_ND_HOST    1
#define VDM_ND_DOCK    2

/* Nintendo dock VDM Message Type */
#define VDM_ND_REQST   0x00
#define VDM_ND_REPLY   0x40

/* Nintendo dock identifiers and limits */
#define DOCK_ID_VOLTAGE_MV  5000u
#define DOCK_ID_CURRENT_MA  500u
#define DOCK_INPUT_VOLTAGE_MV             15000u
#define DOCK_INPUT_CURRENT_LIMIT_MIN_MA   2600u
#define DOCK_INPUT_CURRENT_LIMIT_MAX_MA   3000u

/* Power limits */
#define PD_05V_CHARGING_CURRENT_LIMIT_MA   2000u
#define PD_09V_CHARGING_CURRENT_LIMIT_MA   2000u
#define PD_12V_CHARGING_CURRENT_LIMIT_MA   1500u
#define PD_15V_CHARGING_CURRENT_LIMIT_MA   1200u

#define NON_PD_POWER_RESERVE_UA   2500000u
#define PD_POWER_RESERVE_UA       4500000u

#define PD_INPUT_CURRENT_LIMIT_MIN_MA   0u
#define PD_INPUT_CURRENT_LIMIT_MAX_MA   3000u
#define PD_INPUT_VOLTAGE_LIMIT_MAX_MV   17000u

/* All states with ND are for Nintendo Dock */
enum bm92t_state_type {
	INIT_STATE = 0,
	NEW_PDO,
	PS_RDY_SENT,
	DR_SWAP_SENT,
	VDM_DISC_ID_SENT,
	VDM_ACCEPT_DISC_ID_REPLY,
	VDM_DISC_SVID_SENT,
	VDM_ACCEPT_DISC_SVID_REPLY,
	VDM_DISC_MODE_SENT,
	VDM_ACCEPT_DISC_MODE_REPLY,
	VDM_ENTER_ND_ALT_MODE_SENT,
	VDM_ACCEPT_ENTER_NIN_ALT_MODE_REPLY,
	DP_DISCOVER_MODE,
	DP_QUERY_MODE_SENT,
	DP_ACCEPT_QUERY_MODE_REPLY,
	DP_CFG_START_HPD_SENT,
	VDM_ND_QUERY_DEVICE_SENT,
	VDM_ACCEPT_ND_QUERY_DEVICE_REPLY,
	VDM_ND_ENABLE_USBHUB_SENT,
	VDM_ACCEPT_ND_ENABLE_USBHUB_REPLY,
	VDM_ND_LED_ON_SENT,
	VDM_ACCEPT_ND_LED_ON_REPLY,
	VDM_ND_CUSTOM_CMD_SENT,
	VDM_ACCEPT_ND_CUSTOM_CMD_REPLY,
	VDM_CUSTOM_CMD_SENT,
	VDM_ACCEPT_CUSTOM_CMD_REPLY,
	NINTENDO_CONFIG_HANDLED,
	NORMAL_CONFIG_HANDLED
};

/* Power Data Object, one 32-bit word */
#define PDO_AMP		GENMASK(9, 0)	/* 10mA units */
#define PDO_VOLT	GENMASK(19, 10)	/* 50mV units */
#define PDO_INFO	GENMASK(29, 20)
#define PDO_TYPE	GENMASK(31, 30)

/* Request Data Object, one 32-bit word */
#define RDO_MAX_AMP	GENMASK(9, 0)	/* 10mA units */
#define RDO_OP_AMP	GENMASK(19, 10)	/* 10mA units */
#define RDO_USB_COMMS	BIT(26)
#define RDO_MISMATCH	BIT(27)
#define RDO_OBJ_NO	GENMASK(31, 28)

/* ID Header VDO, first word of a Discover Identity reply */
#define VDO_ID_VID	GENMASK(15, 0)
#define VDO_ID_TYPE	GENMASK(29, 27)
/* Product VDO, third word */
#define VDO_PROD_PID	GENMASK(31, 16)

struct pd_object {
	unsigned int amp;
	unsigned int volt;
	unsigned int info;
	unsigned int type;
};

struct vd_object {
	unsigned int vid;
	unsigned int pid;
	unsigned int type;
};

struct bm92t_device {
	int pdo_no;
	unsigned int charging_limit;
	bool drd_support;
	bool is_nintendo_dock;
	struct pd_object pdo;
	struct vd_object vdo;
};

struct bm92t_platform_data {
	bool dp_disable;
	bool dp_alerts_enable;
	bool dp_signal_toggle_on_resume;
	unsigned int dp_lanes;

	bool led_static_on_suspend;
	bool dock_power_limit_disable;

	unsigned int pd_5v_current_limit;
	unsigned int pd_9v_current_limit;
	unsigned int pd_12v_current_limit;
	unsigned int pd_15v_current_limit;
};

struct bm92t_info {
	struct i2c_client *i2c_client;
	struct bm92t_platform_data *pdata;
	struct gpio_desc *vconn_en_gpio;
	struct work_struct work;
	struct workqueue_struct *event_wq;

	int state;
	unsigned int usbhub_retries;
	bool first_init;
	bool faulted;
	bool sys_reset;

	struct extcon_dev *edev;
	struct delayed_work oneshot_work;
	struct delayed_work power_work;
	struct delayed_work dp_rearm_work;
	unsigned int dp_rearm_retries;
	unsigned int dp_cfg_retries;
	unsigned char dp_cfg[6];
	int dp_valid_lanes;

#ifdef CONFIG_DEBUG_FS
	struct dentry *debugfs_root;
#endif
	struct regulator *batt_chg_reg;
	struct regulator *vbus_src_reg;
	struct regulator *vbus_reg;
	bool vbus_suspended;
	bool pd_charging_enabled;
	unsigned int fw_type;
	unsigned int fw_revision;

	struct bm92t_device cable;

	struct usb_role_switch *role_sw;
};

static const char * const states[] = {
	"INIT_STATE",
	"NEW_PDO",
	"PS_RDY_SENT",
	"DR_SWAP_SENT",
	"VDM_DISC_ID_SENT",
	"VDM_ACCEPT_DISC_ID_REPLY",
	"VDM_DISC_SVID_SENT",
	"VDM_ACCEPT_DISC_SVID_REPLY",
	"VDM_DISC_MODE_SENT",
	"VDM_ACCEPT_DISC_MODE_REPLY",
	"VDM_ENTER_ND_ALT_MODE_SENT",
	"VDM_ACCEPT_ENTER_NIN_ALT_MODE_REPLY",
	"DP_DISCOVER_MODE",
	"DP_QUERY_MODE_SENT",
	"DP_ACCEPT_QUERY_MODE_REPLY",
	"DP_CFG_START_HPD_SENT",
	"VDM_ND_QUERY_DEVICE_SENT",
	"VDM_ACCEPT_ND_QUERY_DEVICE_REPLY",
	"VDM_ND_ENABLE_USBHUB_SENT",
	"VDM_ACCEPT_ND_ENABLE_USBHUB_REPLY",
	"VDM_ND_LED_ON_SENT",
	"VDM_ACCEPT_ND_LED_ON_REPLY",
	"VDM_ND_CUSTOM_CMD_SENT",
	"VDM_ACCEPT_ND_CUSTOM_CMD_REPLY",
	"VDM_CUSTOM_CMD_SENT",
	"VDM_ACCEPT_CUSTOM_CMD_REPLY",
	"NINTENDO_CONFIG_HANDLED",
	"NORMAL_CONFIG_HANDLED"
};

static_assert(ARRAY_SIZE(states) == NORMAL_CONFIG_HANDLED + 1,
	      "states[] out of sync with enum bm92t_state_type");

static const unsigned int bm92t_extcon_cable[] = {
	EXTCON_USB_HOST, /* Id */
	EXTCON_USB,      /* Vbus */
	EXTCON_CHG_USB_PD,   /* USB-PD */
	EXTCON_DISP_DP,  /* DisplayPort. Handled by HPD so not used. */
	EXTCON_NONE
};

struct bm92t_extcon_cables {
	unsigned int cable;
	const char *name;
};

static const struct bm92t_extcon_cables bm92t_extcon_cable_names[] = {
	{ EXTCON_USB_HOST, "USB HOST"},
	{ EXTCON_USB,      "USB"},
	{ EXTCON_CHG_USB_PD,   "USB-PD"},
	{ EXTCON_DISP_DP,  "DisplayPort"},
	{ EXTCON_NONE,     "None"},
	{ -1,              "Unknown"}
};

/* charger current input limits */
static const unsigned int current_input_limits[] = {
	100, 150, 500, 900, 1200, 1500, 2000, 3000
};

/* USB-PD common VDMs */
static const unsigned char vdm_discover_id_msg[6] = {OUTGOING_VDM_REG, 4,
	VDM_CMD_DISC_ID, VDM_STRUCTURED, 0x00, 0xFF};

static const unsigned char vdm_discover_svid_msg[6] = {OUTGOING_VDM_REG, 4,
	VDM_CMD_DISC_SVID, VDM_STRUCTURED, 0x00, 0xFF};

/* DisplayPort Alt Mode */
static const unsigned char vdm_discover_mode_msg[6] = {OUTGOING_VDM_REG, 4,
	VDM_CMD_DISC_MODE, VDM_STRUCTURED, 0x01, 0xFF};

static const unsigned char vdm_enter_nin_alt_mode_msg[6] = {OUTGOING_VDM_REG, 4,
	VDM_CMD_ENTER_MODE, VDM_STRUCTURED | 1, 0x7E, 0x05};

/* Nintendo Dock VDMs */
static const unsigned char vdm_query_device_msg[10] = {OUTGOING_VDM_REG, 8,
	VDM_ND_REQST, VDM_UNSTRUCTURED, 0x7E, 0x05,
	VDM_ND_READ,  VDM_ND_HOST, VDM_NCMD_DEVICE_STATE, 0x00};

static const unsigned char vdm_usbhub_enable_msg[10] = {OUTGOING_VDM_REG, 8,
	VDM_ND_REQST, VDM_UNSTRUCTURED, 0x7E, 0x05,
	VDM_ND_WRITE, VDM_ND_HOST, VDM_NCMD_HUB_CONTROL, 0x00};

/* Payload tail is Fade, Time off, Time on, Duty */
static const unsigned char vdm_usbhub_led_msg[14] = {OUTGOING_VDM_REG, 12,
	VDM_ND_REQST, VDM_UNSTRUCTURED, 0x7E, 0x05,
	VDM_ND_WRITE, VDM_ND_HOST, VDM_NCMD_LED_CONTROL, 0x00,
	0x00, 0x00, 0x00, 0x00};

/* Payload byte 6 is the DP signal disable flag */
static const unsigned char vdm_usbhub_dp_sleep_msg[10] = {OUTGOING_VDM_REG, 8,
	VDM_ND_REQST, VDM_UNSTRUCTURED, 0x7E, 0x05,
	0x00,         VDM_ND_HOST, VDM_NCMD_DP_SIGNAL_DISABLE, 0x00};

static int bm92t_write_reg(struct bm92t_info *info,
			   const unsigned char *buf, unsigned int len)
{
	struct i2c_msg xfer_msg = {
		.addr = info->i2c_client->addr,
		.len = len,
		.buf = (unsigned char *)buf,
	};
	int err;

	dev_dbg(&info->i2c_client->dev,
		"write reg cmd = 0x%02X len = %d\n", buf[0], len);

	err = i2c_transfer(info->i2c_client->adapter, &xfer_msg, 1);
	if (err != 1) {
		dev_err(&info->i2c_client->dev,
			"write to 0x%02X failed: %d\n", buf[0], err);
		return err < 0 ? err : -EIO;
	}

	return 0;
}

static int bm92t_read_reg(struct bm92t_info *info,
			  unsigned char reg, unsigned char *buf, int num)
{
	unsigned char reg_addr = reg;
	struct i2c_msg xfer_msg[2] = {
		{
			.addr = info->i2c_client->addr,
			.len = 1,
			.buf = &reg_addr,
		}, {
			.addr = info->i2c_client->addr,
			.len = num,
			.flags = I2C_M_RD,
			.buf = buf,
		},
	};
	int err;

	err = i2c_transfer(info->i2c_client->adapter, xfer_msg, 2);
	if (err != 2) {
		dev_err(&info->i2c_client->dev,
			"read from 0x%02X failed: %d\n", reg, err);
		return err < 0 ? err : -EIO;
	}

	return 0;
}

/* Most registers are a single little-endian 16-bit word. */
static int bm92t_read_u16(struct bm92t_info *info, unsigned char reg, u16 *val)
{
	__le16 buf;
	int err;

	err = bm92t_read_reg(info, reg, (unsigned char *)&buf, sizeof(buf));
	if (err)
		return err;

	*val = le16_to_cpu(buf);

	return 0;
}

static int bm92t_write_u16(struct bm92t_info *info, unsigned char reg, u16 val)
{
	unsigned char msg[3] = { reg, val & 0xFF, (val >> 8) & 0xFF };

	return bm92t_write_reg(info, msg, sizeof(msg));
}

static int bm92t_send_cmd(struct bm92t_info *info, u16 cmd)
{
	int err;

	err = bm92t_write_u16(info, COMMAND_REG, cmd);
	dev_dbg(&info->i2c_client->dev, "Sent cmd 0x%04X return value %d\n",
		cmd, err);

	return err;
}

/* hdr 0 skips the header check, for unstructured replies */
static int bm92t_read_vdm(struct bm92t_info *info, unsigned char *vdm,
			  unsigned int len, unsigned char hdr)
{
	int err;

	err = bm92t_read_reg(info, INCOMING_VDM_REG, vdm, len);
	if (err)
		return err;

	dev_dbg(&info->i2c_client->dev, "VDM %*ph\n", (int)len, vdm);

	/* vdm[0] is a device-supplied length, it indexes the payload below */
	if (vdm[0] < 4 || vdm[0] > len - 1) {
		dev_err(&info->i2c_client->dev, "Bad VDM length %u\n", vdm[0]);
		return -EPROTO;
	}

	if (hdr && vdm[1] != hdr) {
		dev_err(&info->i2c_client->dev,
			"Unexpected VDM header %02X, wanted %02X\n",
			vdm[1], hdr);
		return -EPROTO;
	}

	return 0;
}

static void bm92t_decode_pdo(struct pd_object *pdo, u32 word)
{
	pdo->amp  = FIELD_GET(PDO_AMP, word);
	pdo->volt = FIELD_GET(PDO_VOLT, word);
	pdo->info = FIELD_GET(PDO_INFO, word);
	pdo->type = FIELD_GET(PDO_TYPE, word);
}

static void bm92t_decode_vdo(struct vd_object *vdo, const unsigned char *buf)
{
	u32 id_header = get_unaligned_le32(buf);
	u32 product = get_unaligned_le32(buf + 8);

	vdo->vid  = FIELD_GET(VDO_ID_VID, id_header);
	vdo->type = FIELD_GET(VDO_ID_TYPE, id_header);
	vdo->pid  = FIELD_GET(VDO_PROD_PID, product);
}

static inline bool bm92t_is_success(const short alert_data)
{
	return (alert_data & ALERT_CMD_DONE);
}

static inline bool bm92t_received_vdm(const short alert_data)
{
	return (alert_data & ALERT_VDM_RECEIVED);
}

static inline bool bm92t_is_plugged(const short status1_data)
{
	return (status1_data & STATUS1_INSERT);
}

static inline bool bm92t_is_ufp(const short status1_data)
{
	return (((status1_data & STATUS1_DR_MASK) >> STATUS1_DR_SHIFT) ==
				DATA_ROLE_UFP);
}

static inline bool bm92t_is_dfp(const short status1_data)
{
	return (((status1_data & STATUS1_DR_MASK) >> STATUS1_DR_SHIFT) ==
				DATA_ROLE_DFP);
}

static inline bool bm92t_is_lastcmd_ok(struct bm92t_info *info,
				       const char *cmd, const short status1_data)
{
	unsigned int lastcmd_status =
		(status1_data & STATUS1_LASTCMD_MASK) >> STATUS1_LASTCMD_SHIFT;

	switch (lastcmd_status) {
	case LASTCMD_COMPLETE:
		break;
	case LASTCMD_ABORTED:
		dev_err(&info->i2c_client->dev, "%s aborted!", cmd);
		break;
	case LASTCMD_INVALID:
		dev_err(&info->i2c_client->dev, "%s invalid!", cmd);
		break;
	case LASTCMD_REJECTED:
		dev_err(&info->i2c_client->dev, "%s rejected!", cmd);
		break;
	case LASTCMD_TERMINATED:
		dev_err(&info->i2c_client->dev, "%s terminated!", cmd);
		break;
	default:
		dev_err(&info->i2c_client->dev, "%s failed! (%d)",
			cmd, lastcmd_status);
	}

	return (lastcmd_status == LASTCMD_COMPLETE);
}

/* The mode VDO is a bare four byte object, not a VDM */
static int bm92t_read_dp_mode_vdo(struct bm92t_info *info, unsigned char *msg)
{
	int err;

	err = bm92t_read_reg(info, INCOMING_VDM_REG, msg, 5);
	if (err)
		return err;

	/* Only the role bits tell a mode VDO from a stale Enter Mode reply */
	if (msg[0] != 4 || !(msg[1] & (VDO_DP_UFP_D | VDO_DP_DFP_D))) {
		dev_dbg(&info->i2c_client->dev,
			"No DP mode VDO published (%*ph)\n", 5, msg);
		return -EPROTO;
	}

	return 0;
}

/* vdo is only read when no pin assignment has been settled on yet */
static int bm92t_handle_dp_config_and_hpd(struct bm92t_info *info,
					  const unsigned char *vdo)
{
	int err = 0, i;
	int valid_lanes = 0;
	unsigned char msg[5] = {};
	unsigned char cfg[6] = {OUTGOING_VDM_REG, 0x04,
				VDO_DP_SUPPORT | VDO_DP_U_UFP_D,
				0x00, 0x00, 0x00};

	/* Set primary pin assignment by lanes supported */
	unsigned char pin_cfg = (info->pdata->dp_lanes == 4) ?
				VDO_DP_PIN_C : VDO_DP_PIN_D;

	if (info->dp_valid_lanes) {
		memcpy(cfg, info->dp_cfg, sizeof(cfg));
		valid_lanes = info->dp_valid_lanes;
		goto configure;
	}

	if (WARN_ON(!vdo))
		return -EINVAL;

	memcpy(msg, vdo, sizeof(msg));

	dev_info(&info->i2c_client->dev,
		 "DP Pin assignments: %02X %02X\n", msg[2], msg[3]);

	/* Prepare UFP_U as UFP_D configuration */
	for (i = 0; i < 2; i++) {
		if (info->cable.is_nintendo_dock) {
			/* Dock reports Plug but uses Receptacle */
			/* Both plug & receptacle pin assignment work, */
			/* because dock ignores them. Use the latter though. */
			if (msg[3] & pin_cfg) {
				cfg[3] = 0x00;
				cfg[4] = pin_cfg;
				valid_lanes = pin_cfg == VDO_DP_PIN_C ? 4 : 2;
				break;
			}
		} else if (!(msg[1] & VDO_DP_RECEPTACLE)) { /* Plug */
			/* Set Plug pin assignment */
			if (msg[2] & pin_cfg) {
				cfg[3] = pin_cfg;
				cfg[4] = 0x00;
				valid_lanes = pin_cfg == VDO_DP_PIN_C ? 4 : 2;
				break;
			}
		} else if (msg[1] & VDO_DP_RECEPTACLE) { /* Receptacle */
			/* Set Receptacle pin assignment */
			if (msg[3] & pin_cfg) {
				cfg[3] = pin_cfg;
				cfg[4] = 0x00;
				valid_lanes = pin_cfg == VDO_DP_PIN_C ? 4 : 2;
				break;
			}
		}

		/* Try secondary pin assignment */
		pin_cfg = (info->pdata->dp_lanes == 4) ?
			  VDO_DP_PIN_D : VDO_DP_PIN_C;
	}

	if (!valid_lanes) {
		dev_warn(&info->i2c_client->dev,
			 "No compatible DP Pin assignment (%d: %02X %02X %02X)!\n",
			 msg[0], msg[1], msg[2], msg[3]);
		return -ENODEV;
	}

	/* Reuse this on the retry path rather than re-reading the mailbox */
	memcpy(info->dp_cfg, cfg, sizeof(info->dp_cfg));
	info->dp_valid_lanes = valid_lanes;

configure:
	/* Send DisplayPort Configuration */
	err = bm92t_write_reg(info, cfg, sizeof(cfg));
	if (err) {
		dev_err(&info->i2c_client->dev, "Writing DP cfg failed!\n");
		return err;
	}

	/* Configure DP Alt mode and start handling HPD */
	return bm92t_send_cmd(info, DP_CFG_AND_START_HPD_CMD);
}

static int bm92t_set_current_limit(struct bm92t_info *info, int max_ua)
{
	int ret = 0;

	dev_info(&info->i2c_client->dev,
		 "Set Charging Current Limit %dma\n", max_ua / 1000);

	if (info->batt_chg_reg) {
		ret = regulator_set_current_limit(info->batt_chg_reg,
						  0, max_ua);
	}

	return ret;
}

static bool bm92t_get_vbus_enabled(struct bm92t_info *info)
{
	bool enabled = false;

	if (info->vbus_reg)
		enabled = regulator_is_enabled(info->vbus_reg);

	return enabled;
}

static int bm92t_set_vbus_enable(struct bm92t_info *info, bool enable)
{
	int ret = 0;
	bool is_enabled;

	dev_dbg(&info->i2c_client->dev,
		"%s VBUS\n", enable ? "Enabling" : "Disabling");

	if (info->vbus_reg) {
		is_enabled = regulator_is_enabled(info->vbus_reg);
		if (enable && !is_enabled)
			ret = regulator_enable(info->vbus_reg);
		else if (!enable && is_enabled)
			ret = regulator_disable(info->vbus_reg);
	}

	return ret;
}

static int bm92t_set_source_mode(struct bm92t_info *info, unsigned int role)
{
	u16 value;
	int err;

	err = bm92t_read_u16(info, CONFIG1_REG, &value);
	if (err)
		return err;

	if (((value & CONFIG1_SPDSRC_MASK) >> CONFIG1_SPDSRC_SHIFT) == role)
		return 0;

	value &= ~CONFIG1_SPDSRC_MASK;
	value |= role << CONFIG1_SPDSRC_SHIFT;

	return bm92t_write_u16(info, CONFIG1_REG, value);
}

static int bm92t_set_dp_alerts(struct bm92t_info *info, bool enable)
{
	return bm92t_write_u16(info, DP_ALERT_EN_REG, enable ? 0xFFFF : 0x0000);
}

static int bm92t_enable_ocp(struct bm92t_info *info)
{
	u16 value;
	int err;

	err = bm92t_read_u16(info, VENDOR_CONFIG_REG, &value);
	if (err)
		return err;

	if (!(value & VENDOR_CONFIG_OCP_DISABLE))
		return 0;

	value &= ~VENDOR_CONFIG_OCP_DISABLE;

	return bm92t_write_u16(info, VENDOR_CONFIG_REG, value);
}

static int bm92t_system_reset_auto(struct bm92t_info *info, bool force)
{
	u16 alert_data, status1_data;
	int err;

	if (force) {
		dev_info(&info->i2c_client->dev, "SYS Reset requested!\n");
		info->faulted = true;
		info->sys_reset = true;
		bm92t_send_cmd(info, SYS_RESET_CMD);
		msleep(33);

		/* Clear alerts, the value is not of interest */
		return bm92t_read_u16(info, ALERT_STATUS_REG, &alert_data);
	}

	err = bm92t_read_u16(info, STATUS1_REG, &status1_data);
	if (err)
		return err;

	if (!bm92t_is_plugged(status1_data) ||
	    bm92t_is_lastcmd_ok(info, "Unknown cmd", status1_data))
		return 0;

	dev_warn(&info->i2c_client->dev, "Stale command status, aborting it\n");
	bm92t_send_cmd(info, ABORT_LASTCMD_SENT_CMD);
	msleep(100);

	/* Clear alerts, the value is not of interest */
	return bm92t_read_u16(info, ALERT_STATUS_REG, &alert_data);
}

static const char *bm92t_extcon_cable_get_name(const unsigned int cable)
{
	int i, count;

	count = ARRAY_SIZE(bm92t_extcon_cable_names);

	for (i = 0; i < count; i++) {
		if (bm92t_extcon_cable_names[i].cable == cable)
			return bm92t_extcon_cable_names[i].name;
	}

	return bm92t_extcon_cable_names[count - 1].name;
}

static void bm92t_usb_role_update(struct bm92t_info *info)
{
	enum usb_role role;

	/* Consider host mode highest priority */
	if (extcon_get_state(info->edev, EXTCON_USB_HOST))
		role = USB_ROLE_HOST;
	else if (extcon_get_state(info->edev, EXTCON_USB))
		role = USB_ROLE_DEVICE;
	else
		role = USB_ROLE_NONE;

	if (usb_role_switch_get_role(info->role_sw) != role)
		usb_role_switch_set_role(info->role_sw, role);
}

static void bm92t_extcon_cable_update(struct bm92t_info *info,
				      const unsigned int cable, bool is_attached)
{
	int state = extcon_get_state(info->edev, cable);

	if (state != is_attached) {
		dev_info(&info->i2c_client->dev, "extcon cable (%02d: %s) %s\n",
			 cable, bm92t_extcon_cable_get_name(cable),
			is_attached ? "attached" : "detached");
		extcon_set_state(info->edev, cable, is_attached);
	}

	switch (cable) {
	case EXTCON_USB:
	case EXTCON_USB_HOST:
		bm92t_usb_role_update(info);
		break;
	default:
		break;
	}
}

/* Advanced from the event workqueue, polled from PM, shutdown and debugfs */
static inline void bm92t_state_machine(struct bm92t_info *info, int state)
{
	WRITE_ONCE(info->state, state);
	dev_dbg(&info->i2c_client->dev, "state = %s\n", states[state]);
}

static void bm92t_calculate_current_limit(struct bm92t_info *info,
					  unsigned int voltage, unsigned int amperage)
{
	struct bm92t_platform_data *pdata = info->pdata;
	unsigned int charging_limit = amperage;
	unsigned int reserve;
	int i;

	reserve = voltage > 5000 ? PD_POWER_RESERVE_UA / voltage :
				   NON_PD_POWER_RESERVE_UA / voltage;

	/* Subtract a USB2 or USB3 port current */
	charging_limit -= min(charging_limit, reserve);

	/* Set limits */
	switch (voltage) {
	case 5000:
		charging_limit = min(charging_limit, pdata->pd_5v_current_limit);
		break;
	case 9000:
		charging_limit = min(charging_limit, pdata->pd_9v_current_limit);
		break;
	case 12000:
		charging_limit = min(charging_limit, pdata->pd_12v_current_limit);
		break;
	case 15000:
	default:
		charging_limit = min(charging_limit, pdata->pd_15v_current_limit);
		break;
	}

	/* Set actual amperage */
	for (i = ARRAY_SIZE(current_input_limits) - 1; i >= 0; i--) {
		if (charging_limit >= current_input_limits[i]) {
			charging_limit = current_input_limits[i];
			break;
		}
	}

	info->cable.charging_limit = charging_limit;
}

static void bm92t_power_work(struct work_struct *work)
{
	struct bm92t_info *info = container_of(to_delayed_work(work),
					       struct bm92t_info, power_work);

	bm92t_set_current_limit(info, info->cable.charging_limit * 1000u);
	info->pd_charging_enabled = true;

	extcon_set_state(info->edev, EXTCON_CHG_USB_PD, true);
}

static void bm92t_extcon_cable_set_init_state(struct work_struct *work)
{
	struct bm92t_info *info = container_of(to_delayed_work(work),
					       struct bm92t_info, oneshot_work);
	u16 status1_data;

	bm92t_set_vbus_enable(info, false);

	/* In case UFP is in an invalid state, request a SYS reset */
	bm92t_system_reset_auto(info, false);

	/* Enable over current protection */
	bm92t_enable_ocp(info);

	/* Enable power to SPDSRC for supporting both OTG and Charger */
	bm92t_set_source_mode(info, SPDSRC12_ON);

	/* Enable DisplayPort Alerts */
	bm92t_set_dp_alerts(info, info->pdata->dp_alerts_enable);

	/* Initialize states for extcons */
	bm92t_extcon_cable_update(info, EXTCON_DISP_DP, false);
	bm92t_extcon_cable_update(info, EXTCON_USB_HOST, false);
	bm92t_extcon_cable_update(info, EXTCON_USB, false);

	dev_info(&info->i2c_client->dev,
		 "extcon cable is set to init state\n");

	msleep(100); /* Wait a bit */

	/* An already-attached port has no alert left to raise. */
	if (!bm92t_read_u16(info, STATUS1_REG, &status1_data) &&
	    bm92t_is_plugged(status1_data))
		bm92t_send_cmd(info, GET_SRC_CAP_CMD);

	queue_work(info->event_wq, &info->work);
}

static bool bm92t_check_pdo(struct bm92t_info *info)
{
	struct device *dev = &info->i2c_client->dev;
	unsigned char pdos[29];
	struct pd_object pdo[7];
	unsigned int prev_wattage = 0;
	unsigned int amperage, voltage, wattage, type;
	int i, err, pdos_no;

	memset(&info->cable, 0, sizeof(struct bm92t_device));

	err = bm92t_read_reg(info, READ_PDOS_SRC_REG, pdos, sizeof(pdos));
	if (err)
		return 0;

	/* pdos[0] is a device-supplied byte count, clamp it to what we read */
	pdos_no = min_t(int, pdos[0], sizeof(pdos) - 1) / sizeof(u32);
	pdos_no = min_t(int, pdos_no, ARRAY_SIZE(pdo));
	if (!pdos_no)
		return 0;

	for (i = 0; i < pdos_no; ++i)
		bm92t_decode_pdo(&pdo[i], get_unaligned_le32(&pdos[1 + i * 4]));

	dev_info(dev, "Supported PDOs:\n");
	for (i = 0; i < pdos_no; ++i) {
		dev_info(dev, "PDO %d: %4dmA %5dmV %s\n",
			 i + 1, pdo[i].amp * 10, pdo[i].volt * 50,
			 (pdo[i].info & PDO_INFO_DR_DATA) ? "DRD" : "No DRD");
	}

	if (pdo[0].info & PDO_INFO_DR_DATA)
		info->cable.drd_support = true;

	/* Check for dock mode */
	if (!info->pdata->dock_power_limit_disable &&
	    pdos_no == 2 &&
	    (pdo[0].volt * 50) == DOCK_ID_VOLTAGE_MV  &&
	    (pdo[0].amp * 10)  == DOCK_ID_CURRENT_MA) {
		/* Only accept 15V, >= 2.6A for dock mode. */
		if (pdo[1].type == PDO_TYPE_FIXED &&
		    (pdo[1].volt * 50) == DOCK_INPUT_VOLTAGE_MV &&
		    (pdo[1].amp * 10)  >= DOCK_INPUT_CURRENT_LIMIT_MIN_MA &&
		    (pdo[1].amp * 10)  <= DOCK_INPUT_CURRENT_LIMIT_MAX_MA) {
			dev_info(dev, "Device in Nintendo mode\n");
			info->cable.pdo_no = 2;
			info->cable.pdo = pdo[1];
			return 1;
		}

		dev_info(dev, "Adapter in dock mode with improper current\n");
		return 0;
	}

	/* Not in dock mode. Check for max possible wattage */
	for (i = 0; i < pdos_no; ++i) {
		type = pdo[i].type;
		voltage = pdo[i].volt * 50;
		amperage = pdo[i].amp * 10;
		wattage = voltage * amperage;

		/* Only USB-PD defined voltages with max 15V. */
		switch (voltage) {
		case 5000:
		case 9000:
		case 12000:
		case 15000:
			break;
		default:
			continue;
		}

		/* Only accept <= 3A and select max wattage with max voltage. */
		if (type == PDO_TYPE_FIXED &&
		    amperage >= PD_INPUT_CURRENT_LIMIT_MIN_MA &&
		    amperage <= PD_INPUT_CURRENT_LIMIT_MAX_MA) {
			if (wattage > prev_wattage ||
			    (voltage > (info->cable.pdo.volt * 50) &&
			    wattage && wattage == prev_wattage) ||
			   (!info->cable.pdo_no && !amperage && voltage == 5000)) {
				prev_wattage = wattage;
				info->cable.pdo_no = i + 1;
				info->cable.pdo = pdo[i];
			}
		}
	}

	if (info->cable.pdo_no) {
		dev_info(&info->i2c_client->dev, "Device in powered mode\n");
		return 1;
	}

	return 0;
}

static int bm92t_send_rdo(struct bm92t_info *info)
{
	unsigned char msg[6] = { SET_RDO_REG, 0x04 };
	u32 rdo;
	int err;

	/* Calculate operating current */
	bm92t_calculate_current_limit(info, info->cable.pdo.volt * 50,
				      info->cable.pdo.amp * 10);

	dev_info(&info->i2c_client->dev,
		 "Requesting %d: min %dmA, max %4dmA, %5dmV\n",
		 info->cable.pdo_no, info->cable.charging_limit,
		 info->cable.pdo.amp * 10,
		 info->cable.pdo.volt * 50);

	rdo = RDO_USB_COMMS |
	      FIELD_PREP(RDO_OBJ_NO, info->cable.pdo_no) |
	      FIELD_PREP(RDO_MAX_AMP, info->cable.pdo.amp) |
	      FIELD_PREP(RDO_OP_AMP, info->cable.charging_limit / 10);

	put_unaligned_le32(rdo, &msg[2]);

	err = bm92t_write_reg(info, msg, sizeof(msg));
	if (err) {
		dev_err(&info->i2c_client->dev, "Send RDO failure!\n");
		return err;
	}

	return bm92t_send_cmd(info, SEND_RDO_CMD);
}

static int bm92t_send_vdm(struct bm92t_info *info, const unsigned char *msg,
			  unsigned int len)
{
	int err;

	err = bm92t_write_reg(info, msg, len);
	if (err) {
		dev_err(&info->i2c_client->dev, "Send VDM failure!\n");
		return err;
	}

	return bm92t_send_cmd(info, SEND_VDM_CMD);
}

static void bm92t_usbhub_led_cfg(struct bm92t_info *info,
				 unsigned char duty, unsigned char time_on,
	unsigned char time_off, unsigned char fade)
{
	unsigned char msg[sizeof(vdm_usbhub_led_msg)];

	memcpy(msg, vdm_usbhub_led_msg, sizeof(msg));
	msg[10] = fade;
	msg[11] = time_off;
	msg[12] = time_on;
	msg[13] = duty;

	bm92t_send_vdm(info, msg, sizeof(msg));
}

static void bm92t_usbhub_led_cfg_wait(struct bm92t_info *info,
				      unsigned char duty, unsigned char time_on,
	unsigned char time_off, unsigned char fade)
{
	int retries = BM92T_VDM_WAIT_RETRIES;

	if (READ_ONCE(info->state) != NINTENDO_CONFIG_HANDLED)
		return;

	bm92t_state_machine(info, VDM_ND_CUSTOM_CMD_SENT);
	bm92t_usbhub_led_cfg(info, duty, time_on, time_off, fade);

	while (READ_ONCE(info->state) != NINTENDO_CONFIG_HANDLED &&
	       retries-- > 0)
		usleep_range(1000, 2000);
}

static bool bm92t_config_handled(struct bm92t_info *info)
{
	int state = READ_ONCE(info->state);

	return state == NINTENDO_CONFIG_HANDLED || state == NORMAL_CONFIG_HANDLED;
}

static void bm92t_usbhub_dp_sleep(struct bm92t_info *info, bool sleep)
{
	unsigned char msg[sizeof(vdm_usbhub_dp_sleep_msg)];
	int retries = BM92T_VDM_WAIT_RETRIES;

	if (!bm92t_config_handled(info))
		return;

	if (READ_ONCE(info->state) == NINTENDO_CONFIG_HANDLED)
		bm92t_state_machine(info, VDM_ND_CUSTOM_CMD_SENT);
	else
		bm92t_state_machine(info, VDM_CUSTOM_CMD_SENT);

	memcpy(msg, vdm_usbhub_dp_sleep_msg, sizeof(msg));
	msg[6] = sleep ? 1 : 0;

	bm92t_send_vdm(info, msg, sizeof(msg));

	while (!bm92t_config_handled(info) && retries-- > 0)
		usleep_range(1000, 2000);
}

static void bm92t_dp_rearm_work_fn(struct work_struct *work)
{
	struct bm92t_info *info = container_of(to_delayed_work(work),
					       struct bm92t_info, dp_rearm_work);
	u16 dp_data;
	int err;

	if (!bm92t_config_handled(info)) {
		if (info->dp_rearm_retries--)
			schedule_delayed_work(&info->dp_rearm_work,
					      msecs_to_jiffies(500));
		return;
	}

	err = bm92t_read_u16(info, DP_STATUS_REG, &dp_data);
	if (err)
		return;

	if (!(dp_data & DP_STATUS_DP_EN) || dp_data & DP_STATUS_SIGNAL_ON)
		return;

	dev_info(&info->i2c_client->dev,
		 "DP alt mode up but signal off (DP_STATUS %04X), waking it\n",
		 dp_data);

	bm92t_usbhub_dp_sleep(info, false);
}

static void bm92t_print_dp_dev_info(struct device *dev,
				    struct vd_object *vdo)
{
	dev_info(dev, "Connected PD device:\n");
	dev_info(dev, "VID: %04X, PID: %04X\n", vdo->vid, vdo->pid);

	switch (vdo->type) {
	case VDO_ID_TYPE_NONE:
		dev_info(dev, "Type: Undefined\n");
		break;
	case VDO_ID_TYPE_PD_HUB:
		dev_info(dev, "Type: PD HUB\n");
		break;
	case VDO_ID_TYPE_PD_PERIPH:
		dev_info(dev, "Type: PD Peripheral\n");
		break;
	case VDO_ID_TYPE_PASS_CBL:
		dev_info(dev, "Type: Passive Cable\n");
		break;
	case VDO_ID_TYPE_ACTI_CBL:
		dev_info(dev, "Type: Active Cable\n");
		break;
	case VDO_ID_TYPE_ALTERNATE:
		dev_info(dev, "Type: Alternate Mode Adapter\n");
		break;
	default:
		dev_info(dev, "Type: Unknown (%d)\n", vdo->type);
		break;
	}
}

static void bm92t_dp_attached(struct bm92t_info *info)
{
	if (info->cable.is_nintendo_dock) {
		bm92t_send_vdm(info, vdm_query_device_msg,
			       sizeof(vdm_query_device_msg));
		bm92t_state_machine(info, VDM_ND_QUERY_DEVICE_SENT);
	} else {
		bm92t_state_machine(info, NORMAL_CONFIG_HANDLED);
	}

	bm92t_extcon_cable_update(info, EXTCON_DISP_DP, true);

	info->dp_rearm_retries = 10;
	schedule_delayed_work(&info->dp_rearm_work, msecs_to_jiffies(500));
}

static void bm92t_dp_configure(struct bm92t_info *info,
			       const unsigned char *vdo)
{
	info->dp_cfg_retries = 3;
	info->dp_valid_lanes = 0;

	bm92t_state_machine(info, bm92t_handle_dp_config_and_hpd(info, vdo) ?
				  INIT_STATE : DP_CFG_START_HPD_SENT);
}

static void bm92t_event_handler(struct work_struct *work)
{
	struct bm92t_info *info = container_of(work, struct bm92t_info, work);
	struct device *dev = &info->i2c_client->dev;
	struct pd_object curr_pdo;
	u16 alert_data, status1_data, status2_data, dp_data;
	unsigned char vdm[29], pdo[5], rdo[5], dp_vdo[5];
	unsigned int op_amp, max_amp;
	int i, err, fault;
	u32 word;

	/* Read status registers at 02h, 03h and 04h */
	err = bm92t_read_u16(info, ALERT_STATUS_REG, &alert_data);
	if (err)
		goto ret;
	err = bm92t_read_u16(info, STATUS1_REG, &status1_data);
	if (err)
		goto ret;
	err = bm92t_read_u16(info, STATUS2_REG, &status2_data);
	if (err)
		goto ret;
	err = bm92t_read_u16(info, DP_STATUS_REG, &dp_data);
	if (err)
		goto ret;

	dev_dbg(dev,
		"Alert= 0x%04X Status1= 0x%04X Status2= 0x%04X DP= 0x%04X State= %s\n",
		alert_data, status1_data, status2_data,
		dp_data, states[info->state]);

	/* Report sink error */
	if (alert_data & ALERT_SNK_FAULT)
		dev_err(dev, "Sink fault occurred!\n");

	/* Report source error */
	if (alert_data & ALERT_SRC_FAULT)
		dev_err(dev, "Source fault occurred!\n");

	/* TODO: DP event handling */
	if (alert_data == ALERT_DP_EVENT)
		goto ret;

	fault = status1_data & STATUS1_FAULT_MASK;
	if (fault) {
		if (info->faulted)
			goto ret;

		/* Resetting with anything attached kills detection for good. */
		if (status1_data & (STATUS1_INSERT | STATUS1_VSAFE))
			goto ret;

		info->faulted = true;
		dev_err(dev, "Internal error occurred. Ecode = %d\n", fault);
		bm92t_state_machine(info, INIT_STATE);
		bm92t_extcon_cable_update(info, EXTCON_DISP_DP, false);
		bm92t_extcon_cable_update(info, EXTCON_USB_HOST, false);
		bm92t_extcon_cable_update(info, EXTCON_USB, false);
		bm92t_set_vbus_enable(info, false);
		bm92t_system_reset_auto(info, true);
		goto ret;
	}

	info->faulted = false;

	if (info->sys_reset) {
		info->sys_reset = false;
		msleep(100);

		/* Enable over current protection */
		bm92t_enable_ocp(info);

		/* Enable power to SPDSRC for supporting both OTG and Charger */
		bm92t_set_source_mode(info, SPDSRC12_ON);
	}

	if (alert_data & ALERT_SRC_FAULT &&
	    status1_data & STATUS1_SRC_MODE) {
		bm92t_send_cmd(info, PD_HARD_RST_CMD);
		goto src_fault;
	}

	/* Check if cable removed */
	if (alert_data & ALERT_PLUGPULL) {
		if (!bm92t_is_plugged(status1_data)) { /* Pull event */
src_fault:
			/* Cancel any pending charging enable work */
			cancel_delayed_work(&info->power_work);

			/* Disable VBUS in case it's enabled */
			bm92t_set_vbus_enable(info, false);

			/* Disable charging */
			if (info->pd_charging_enabled) {
				bm92t_set_current_limit(info, 0);
				info->pd_charging_enabled = false;
				bm92t_extcon_cable_update(info,
							  EXTCON_CHG_USB_PD, false);
			}

			/* Reset USB modes and state */
			info->usbhub_retries = BM92T_USBHUB_RETRIES;
			bm92t_extcon_cable_update(info, EXTCON_DISP_DP, false);
			bm92t_extcon_cable_update(info, EXTCON_USB_HOST, false);
			bm92t_extcon_cable_update(info, EXTCON_USB, false);
			bm92t_state_machine(info, INIT_STATE);
			goto ret;
		} else if (status1_data & STATUS1_SRC_MODE && /* OTG plug event */
			   status2_data & STATUS2_OTG_INSERT) {
			/* Enable VBUS for sourcing power to OTG device */
			bm92t_set_vbus_enable(info, true);

			/* Set USB to host mode */
			bm92t_extcon_cable_update(info, EXTCON_USB, false);
			bm92t_extcon_cable_update(info, EXTCON_USB_HOST, true);
			goto ret;
		} else if (alert_data & ALERT_CONTRACT && !info->first_init) {
			/* When there's a plug-in wake-up, check if a new
			 * contract was received. If yes continue with init.
			 *
			 * In case of no new PDO, wait for it.
			 * Otherwise PD will fail.
			 * In case of non-PD charger, this doesn't affect the
			 * result.
			 */
			if (!(alert_data & ALERT_PDO))
				msleep(500);
		} else {
			/* Simple plug event */
			goto ret;
		}
	}

	switch (info->state) {
	case INIT_STATE:
		if (alert_data & ALERT_SRC_PLUGIN) {
			dev_info(dev, "Device in OTG mode\n");
			info->first_init = false;
			if (bm92t_is_dfp(status1_data)) {
				/* Reset cable info */
				memset(&info->cable, 0,
				       sizeof(struct bm92t_device));

				bm92t_send_vdm(info, vdm_discover_id_msg,
					       sizeof(vdm_discover_id_msg));
				bm92t_state_machine(info, VDM_DISC_ID_SENT);
			}
			break;
		}

		if (status1_data & STATUS1_SRC_MODE &&
		    status2_data & STATUS2_OTG_INSERT) {
			info->first_init = false;
			dev_info(dev, "Device in OTG mode (no alert)\n");
			break;
		}

		if ((alert_data & ALERT_CONTRACT) || info->first_init) {
			/* Exit if unplugged */
			if (!bm92t_is_plugged(status1_data))
				goto init_contract_out;

			/* Check if sink mode is enabled for first init */
			/* If not, exit and wait for next alert */
			if (info->first_init &&
			    !(alert_data & ALERT_CONTRACT) &&
			    !(status1_data & STATUS1_SPDSNK)) {
				goto init_contract_out;
			}

			/* Negotiate new power profile */
			if (!bm92t_check_pdo(info)) {
				dev_info(dev, "Power Negotiation not supported\n");
				bm92t_state_machine(info, INIT_STATE);
				msleep(550); /* WAR: charger good power test */
				bm92t_extcon_cable_update(info,
							  EXTCON_USB, true);
				goto init_contract_out;
			}

			/* Power negotiation succeeded */
			bm92t_send_rdo(info);
			bm92t_state_machine(info, NEW_PDO);
			msleep(20);

init_contract_out:
			info->first_init = false;
			break;
		}

		/* Check if forced workqueue and unplugged */
		if (!alert_data && !bm92t_is_plugged(status1_data))
			bm92t_extcon_cable_update(info, EXTCON_USB, false);
		break;

	case NEW_PDO:
		if (bm92t_is_success(alert_data))
			dev_dbg(dev, "cmd done in NEW_PDO state\n");

		if (alert_data & ALERT_CONTRACT) {
			/* Check PDO/RDO */
			err = bm92t_read_reg(info, CURRENT_PDO_REG,
					     pdo, sizeof(pdo));
			if (err)
				break;
			bm92t_decode_pdo(&curr_pdo, get_unaligned_le32(&pdo[1]));

			err = bm92t_read_reg(info, CURRENT_RDO_REG,
					     rdo, sizeof(rdo));
			if (err)
				break;
			word = get_unaligned_le32(&rdo[1]);
			op_amp = FIELD_GET(RDO_OP_AMP, word);
			max_amp = FIELD_GET(RDO_MAX_AMP, word);

			dev_info(dev, "New PD Contract:\n");
			dev_info(dev, "PDO: %d: %dmA, %dmV\n",
				 info->cable.pdo_no, curr_pdo.amp * 10,
				curr_pdo.volt * 50);
			dev_info(dev, "RDO: op %dmA, %dmA max\n",
				 op_amp * 10, max_amp * 10);

			if (word & RDO_MISMATCH)
				dev_err(dev, "PD mismatch!\n");

			if ((!op_amp && !max_amp) ||
			    curr_pdo.volt != info->cable.pdo.volt)
				dev_warn(dev,
					 "Contract settled at %dmV op %dmA, requested %dmV\n",
					 curr_pdo.volt * 50, op_amp * 10,
					 info->cable.pdo.volt * 50);

			bm92t_send_cmd(info, PS_RDY_CMD);
			bm92t_state_machine(info, PS_RDY_SENT);
		}
		break;

	case PS_RDY_SENT:
		if (bm92t_is_success(alert_data)) {
			bm92t_extcon_cable_update(info, EXTCON_USB_HOST, true);
			schedule_delayed_work(&info->power_work,
					      msecs_to_jiffies(2000));

			if (bm92t_is_ufp(status1_data)) {
				/* Check if Dual-Role Data is supported */
				if (!info->cable.drd_support) {
					dev_err(dev, "Device in UFP and DRD not supported!\n");
					break;
				}

				bm92t_send_cmd(info, DR_SWAP_CMD);
				bm92t_state_machine(info, DR_SWAP_SENT);
			} else if (bm92t_is_dfp(status1_data)) {
				dev_dbg(dev, "Already in DFP mode\n");
				bm92t_send_vdm(info, vdm_discover_id_msg,
					       sizeof(vdm_discover_id_msg));
				bm92t_state_machine(info, VDM_DISC_ID_SENT);
			}
		}
		break;

	case DR_SWAP_SENT:
		if (bm92t_is_success(alert_data) &&
		    bm92t_is_plugged(status1_data) &&
		    bm92t_is_lastcmd_ok(info, "DR_SWAP_CMD", status1_data) &&
		    bm92t_is_dfp(status1_data)) {
			bm92t_send_vdm(info, vdm_discover_id_msg,
				       sizeof(vdm_discover_id_msg));
			bm92t_state_machine(info, VDM_DISC_ID_SENT);
		}
		break;

	case VDM_DISC_ID_SENT:
		if (bm92t_received_vdm(alert_data)) {
			bm92t_send_cmd(info, ACCEPT_VDM_CMD);
			bm92t_state_machine(info, VDM_ACCEPT_DISC_ID_REPLY);
		} else if (bm92t_is_success(alert_data)) {
			dev_dbg(dev, "cmd done in VDM_DISC_ID_SENT\n");
		}
		break;

	case VDM_ACCEPT_DISC_ID_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Check incoming VDM */
			err = bm92t_read_vdm(info, vdm, sizeof(vdm),
					     VDM_ACK | VDM_CMD_DISC_ID);
			if (err || vdm[0] < 16)
				break;

			bm92t_decode_vdo(&info->cable.vdo, &vdm[5]);
			bm92t_print_dp_dev_info(dev, &info->cable.vdo);

			/* Check if Nintendo dock. */
			if (!(info->cable.vdo.type == VDO_ID_TYPE_ALTERNATE &&
			      info->cable.vdo.vid == VID_NINTENDO &&
			      info->cable.vdo.pid == PID_NIN_DOCK)) {
				dev_info(dev, "VID/PID not Nintendo Dock\n");

				if (info->pdata->dp_disable) {
					bm92t_state_machine(info,
							    NORMAL_CONFIG_HANDLED);
					break;
				}

				bm92t_send_vdm(info, vdm_discover_svid_msg,
					       sizeof(vdm_discover_svid_msg));
				bm92t_state_machine(info, VDM_DISC_SVID_SENT);
			} else {
				info->cable.is_nintendo_dock = true;
				bm92t_send_vdm(info, vdm_enter_nin_alt_mode_msg,
					       sizeof(vdm_enter_nin_alt_mode_msg));
				bm92t_state_machine(info, VDM_ENTER_ND_ALT_MODE_SENT);
			}
		}
		break;

	case VDM_DISC_SVID_SENT:
		if (bm92t_received_vdm(alert_data)) {
			bm92t_send_cmd(info, ACCEPT_VDM_CMD);
			bm92t_state_machine(info, VDM_ACCEPT_DISC_SVID_REPLY);
		} else if (bm92t_is_success(alert_data)) {
			dev_dbg(dev, "cmd done in VDM_DISC_SVID_SENT\n");
		}
		break;

	case VDM_ACCEPT_DISC_SVID_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Check discovered SVIDs */
			err = bm92t_read_vdm(info, vdm, sizeof(vdm),
					     VDM_ACK | VDM_CMD_DISC_SVID);
			if (err)
				break;

			dev_info(dev, "Supported SVIDs:\n");
			for (i = 0; i < ((vdm[0] - 4) / 2); i++)
				dev_info(dev, "SVID%d %04X\n", i,
					 get_unaligned_le16(&vdm[5 + i * 2]));

			/* Request DisplayPort Alt mode support SVID (0xFF01) */
			bm92t_send_vdm(info, vdm_discover_mode_msg,
				       sizeof(vdm_discover_mode_msg));
			bm92t_state_machine(info, VDM_DISC_MODE_SENT);
		}
		break;

	case VDM_DISC_MODE_SENT:
		if (bm92t_received_vdm(alert_data)) {
			bm92t_send_cmd(info, ACCEPT_VDM_CMD);
			bm92t_state_machine(info, VDM_ACCEPT_DISC_MODE_REPLY);
		} else if (bm92t_is_success(alert_data)) {
			dev_dbg(dev, "cmd done in VDM_DISC_MODE_SENT\n");
		}
		break;

	case VDM_ACCEPT_DISC_MODE_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Check incoming VDM */
			err = bm92t_read_vdm(info, vdm, sizeof(vdm),
					     VDM_ACK | VDM_CMD_DISC_MODE);
			if (err)
				break;

			/* Check if DisplayPort Alt mode is supported */
			if (vdm[0] > 4 && /* Has VDO objects */
			    vdm[2] == VDM_STRUCTURED &&
			    vdm[3] == 0x01 && vdm[4] == 0xFF && /* SVID DP */
			    vdm[5] & VDO_DP_UFP_D &&
			    vdm[5] & VDO_DP_SUPPORT) {
				dev_info(dev, "DisplayPort Alt Mode supported");
				for (i = 0; i < ((vdm[0] - 4) / 4); i++)
					dev_info(dev, "DPCap%d %08X\n", i,
						 get_unaligned_le32(&vdm[5 + i * 4]));

				/* Enter automatic DisplayPort handling */
				bm92t_send_cmd(info, DP_ENTER_MODE_CMD);
				msleep(100); /* WAR: may not need to wait */
				bm92t_state_machine(info, DP_DISCOVER_MODE);
			}
		}
		break;

	case VDM_ENTER_ND_ALT_MODE_SENT:
		if (bm92t_received_vdm(alert_data)) {
			bm92t_send_cmd(info, ACCEPT_VDM_CMD);
			bm92t_state_machine(info, VDM_ACCEPT_ENTER_NIN_ALT_MODE_REPLY);
		} else if (bm92t_is_success(alert_data)) {
			dev_dbg(dev, "cmd done in VDM_ENTER_ND_ALT_MODE_SENT\n");
		}
		break;

	case VDM_ACCEPT_ENTER_NIN_ALT_MODE_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Check incoming VDM */
			err = bm92t_read_vdm(info, vdm, sizeof(vdm),
					     VDM_ACK | VDM_CMD_ENTER_MODE);

			/* Check if supported. */
			if (err || vdm[2] != (VDM_STRUCTURED | 1) ||
			    vdm[3] != 0x7e || vdm[4] != 0x05) {
				dev_err(dev, "Failed to enter Nintendo Alt Mode!\n");
				break;
			}

			if (info->pdata->dp_disable) {
				bm92t_send_vdm(info, vdm_query_device_msg,
					       sizeof(vdm_query_device_msg));
				bm92t_state_machine(info,
						    VDM_ND_QUERY_DEVICE_SENT);
				break;
			}

			/* Enter automatic DisplayPort handling */
			bm92t_send_cmd(info, DP_ENTER_MODE_CMD);
			msleep(100); /* WAR: may not need to wait */
			bm92t_state_machine(info, DP_DISCOVER_MODE);
		}
		break;

	case DP_DISCOVER_MODE:
		if (bm92t_is_success(alert_data)) {
			/*
			 * A link still signalling outlived the driver, so the
			 * chip is already configured. Reconfiguring it here
			 * drops the signal and blanks the display.
			 */
			if (dp_data & DP_STATUS_SIGNAL_ON) {
				dev_info(dev, "DP link already up, adopting\n");
				bm92t_dp_attached(info);
				break;
			}

			/* Ask the port itself when the mailbox holds no VDO */
			if (bm92t_read_dp_mode_vdo(info, dp_vdo)) {
				bm92t_send_vdm(info, vdm_discover_mode_msg,
					       sizeof(vdm_discover_mode_msg));
				bm92t_state_machine(info, DP_QUERY_MODE_SENT);
				break;
			}

			bm92t_dp_configure(info, dp_vdo);
		}
		break;

	case DP_QUERY_MODE_SENT:
		if (bm92t_received_vdm(alert_data)) {
			bm92t_send_cmd(info, ACCEPT_VDM_CMD);
			bm92t_state_machine(info, DP_ACCEPT_QUERY_MODE_REPLY);
		} else if (bm92t_is_success(alert_data)) {
			dev_dbg(dev, "cmd done in DP_QUERY_MODE_SENT\n");
		}
		break;

	case DP_ACCEPT_QUERY_MODE_REPLY:
		if (bm92t_is_success(alert_data)) {
			err = bm92t_read_vdm(info, vdm, sizeof(vdm),
					     VDM_ACK | VDM_CMD_DISC_MODE);
			if (err || vdm[0] < 8) {
				dev_err(dev, "No DP mode VDO discovered!\n");
				bm92t_state_machine(info, INIT_STATE);
				break;
			}

			/* The VDO the mailbox failed to publish */
			dp_vdo[0] = 4;
			memcpy(&dp_vdo[1], &vdm[5], 4);

			bm92t_dp_configure(info, dp_vdo);
		}
		break;

	case DP_CFG_START_HPD_SENT:
		if (bm92t_is_success(alert_data)) {
			if (bm92t_is_plugged(status1_data) &&
			    bm92t_is_lastcmd_ok(info, "DP_CFG_AND_START_HPD_CMD",
						status1_data)) {
				bm92t_dp_attached(info);
			} else if (bm92t_is_plugged(status1_data)) {
				if (info->dp_cfg_retries--)
					err = bm92t_handle_dp_config_and_hpd(info, NULL);
				else
					err = -ETIMEDOUT;

				if (err)
					bm92t_state_machine(info, INIT_STATE);
			}
		}
		break;

	/* Nintendo Dock VDMs */
	case VDM_ND_QUERY_DEVICE_SENT:
		if (bm92t_received_vdm(alert_data)) {
			bm92t_send_cmd(info, ACCEPT_VDM_CMD);
			bm92t_state_machine(info, VDM_ACCEPT_ND_QUERY_DEVICE_REPLY);
		} else if (bm92t_is_success(alert_data)) {
			dev_dbg(dev, "cmd done in VDM_ND_QUERY_DEVICE_SENT\n");
		}
		break;

	case VDM_ACCEPT_ND_QUERY_DEVICE_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Check incoming VDM */
			err = bm92t_read_vdm(info, vdm, sizeof(vdm), 0);

			if (!err && vdm[0] >= 12 && vdm[6] == VDM_ND_DOCK &&
			    vdm[7] == (VDM_NCMD_DEVICE_STATE + 1)) {
				/* Check if USB HUB is supported */
				if (vdm[11] & 0x02) {
					bm92t_extcon_cable_update(info,
								  EXTCON_USB_HOST, false);
					msleep(500);
					bm92t_extcon_cable_update(info,
								  EXTCON_USB, true);
					dev_warn(dev, "Dock has old FW!\n");
				}
				dev_info(dev, "Dock state: %02X %02X %02X %02X\n",
					 vdm[9], vdm[10], vdm[11], vdm[12]);
			} else {
				dev_err(dev, "Failed to get dock state reply!");
			}

			/* Set dock LED */
			bm92t_usbhub_led_cfg(info, 128, 0, 0, 64);
			bm92t_state_machine(info, VDM_ND_LED_ON_SENT);
		}
		break;

	case VDM_ND_LED_ON_SENT:
		if (bm92t_received_vdm(alert_data)) {
			bm92t_send_cmd(info, ACCEPT_VDM_CMD);
			bm92t_state_machine(info, VDM_ACCEPT_ND_LED_ON_REPLY);
		} else if (bm92t_is_success(alert_data)) {
			dev_dbg(dev, "cmd done in VDM_ND_LED_ON_SENT\n");
		}
		break;

	case VDM_ACCEPT_ND_LED_ON_REPLY:
		if (bm92t_is_success(alert_data)) {
			msleep(500); /* Wait for hub to power up */
			bm92t_send_vdm(info, vdm_usbhub_enable_msg,
				       sizeof(vdm_usbhub_enable_msg));
			bm92t_state_machine(info, VDM_ND_ENABLE_USBHUB_SENT);
		}
		break;

	case VDM_ND_ENABLE_USBHUB_SENT:
		if (bm92t_received_vdm(alert_data)) {
			bm92t_send_cmd(info, ACCEPT_VDM_CMD);
			bm92t_state_machine(info, VDM_ACCEPT_ND_ENABLE_USBHUB_REPLY);
		} else if (bm92t_is_success(alert_data)) {
			dev_dbg(dev, "cmd done in VDM_ND_ENABLE_USBHUB_SENT\n");
		}
		break;

	case VDM_ACCEPT_ND_ENABLE_USBHUB_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Check incoming VDM */
			err = bm92t_read_vdm(info, vdm, sizeof(vdm), 0);

			if (!err && vdm[0] >= 8 && vdm[6] == VDM_ND_DOCK &&
			    vdm[7] == (VDM_NCMD_HUB_CONTROL + 1) &&
			    info->usbhub_retries) {
				if (vdm[5] & VDM_ND_BUSY) {
					msleep(250);
					dev_info(dev, "Retrying USB HUB enable...\n");
					bm92t_send_vdm(info,
						       vdm_usbhub_enable_msg,
						sizeof(vdm_usbhub_enable_msg));
					bm92t_state_machine(info,
							    VDM_ND_ENABLE_USBHUB_SENT);
					info->usbhub_retries--;
					break;
				}
			} else if (!info->usbhub_retries) {
				dev_err(dev, "USB HUB enable timed out!\n");
			} else {
				dev_err(dev, "USB HUB enable failed!\n");
			}

			bm92t_state_machine(info, NINTENDO_CONFIG_HANDLED);
		}
		break;

	case VDM_ND_CUSTOM_CMD_SENT:
		if (bm92t_received_vdm(alert_data)) {
			bm92t_send_cmd(info, ACCEPT_VDM_CMD);
			bm92t_state_machine(info, VDM_ACCEPT_ND_CUSTOM_CMD_REPLY);
		} else if (bm92t_is_success(alert_data)) {
			dev_dbg(dev, "cmd done in VDM_ND_CUSTOM_CMD_SENT\n");
		}
		break;

	case VDM_ACCEPT_ND_CUSTOM_CMD_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Drain the reply, its content is not of interest */
			bm92t_read_vdm(info, vdm, sizeof(vdm), 0);
			bm92t_state_machine(info, NINTENDO_CONFIG_HANDLED);
		}
		break;
	/* End of Nintendo Dock VDMs */

	case VDM_CUSTOM_CMD_SENT:
		if (bm92t_received_vdm(alert_data)) {
			bm92t_send_cmd(info, ACCEPT_VDM_CMD);
			bm92t_state_machine(info, VDM_ACCEPT_CUSTOM_CMD_REPLY);
		} else if (bm92t_is_success(alert_data)) {
			dev_dbg(dev, "cmd done in VDM_CUSTOM_CMD_SENT\n");
		}
		break;

	case VDM_ACCEPT_CUSTOM_CMD_REPLY:
		if (bm92t_is_success(alert_data)) {
			/* Drain the reply, its content is not of interest */
			bm92t_read_vdm(info, vdm, sizeof(vdm), 0);
			bm92t_state_machine(info, NORMAL_CONFIG_HANDLED);
		}
		break;

	case NORMAL_CONFIG_HANDLED:
	case NINTENDO_CONFIG_HANDLED:
		break;

	default:
		dev_err(dev, "Invalid state!\n");
		break;
	}

ret:
	enable_irq(info->i2c_client->irq);
}

static irqreturn_t bm92t_interrupt_handler(int irq, void *handle)
{
	struct bm92t_info *info = handle;

	disable_irq_nosync(info->i2c_client->irq);
	queue_work(info->event_wq, &info->work);
	return IRQ_HANDLED;
}

static void bm92t_remove(struct i2c_client *client)
{
	struct bm92t_info *info = i2c_get_clientdata(client);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(info->debugfs_root);
#endif
	disable_irq(client->irq);

	cancel_delayed_work_sync(&info->oneshot_work);
	cancel_delayed_work_sync(&info->power_work);
	cancel_delayed_work_sync(&info->dp_rearm_work);
	cancel_work_sync(&info->work);

	free_irq(client->irq, info);

	destroy_workqueue(info->event_wq);

	if (info->vbus_src_reg && regulator_is_enabled(info->vbus_src_reg))
		regulator_disable(info->vbus_src_reg);

	usb_role_switch_put(info->role_sw);
}

static void bm92t_shutdown(struct i2c_client *client)
{
	struct bm92t_info *info = i2c_get_clientdata(client);

	/* Disable Dock LED if enabled */
	bm92t_usbhub_led_cfg_wait(info, 0, 0, 0, 128);

	/* Disable SPDSRC */
	bm92t_set_source_mode(info, SPDSRC12_OFF);

	gpiod_set_value_cansleep(info->vconn_en_gpio, 0);

	if (info->vbus_src_reg && regulator_is_enabled(info->vbus_src_reg))
		regulator_disable(info->vbus_src_reg);

	/* Disable DisplayPort Alerts */
	if (info->pdata->dp_alerts_enable)
		bm92t_set_dp_alerts(info, false);
}

#ifdef CONFIG_DEBUG_FS
static int bm92t_regs_print(struct seq_file *s, const char *reg_name,
			    unsigned char reg_addr, int size)
{
	struct bm92t_info *info = (struct bm92t_info *)(s->private);
	unsigned char msg[5];
	u16 reg_val16;
	int err;

	switch (size) {
	case 2:
		err = bm92t_read_u16(info, reg_addr, &reg_val16);
		if (!err)
			seq_printf(s, "%s 0x%04X\n", reg_name, reg_val16);
		break;
	case 5:
		err = bm92t_read_reg(info, reg_addr, msg, sizeof(msg));
		if (!err)
			seq_printf(s, "%s 0x%02X%02X%02X%02X\n",
				   reg_name, msg[4], msg[3], msg[2], msg[1]);
		break;
	default:
		err = -EINVAL;
		break;
	}

	if (err)
		dev_err(&info->i2c_client->dev, "Cannot read 0x%02X\n", reg_addr);

	return err;
}

static int bm92t_regs_show(struct seq_file *s, void *data)
{
	int err;

	err = bm92t_regs_print(s, "ALERT_STATUS:  ", ALERT_STATUS_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "STATUS1:       ", STATUS1_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "STATUS2:       ", STATUS2_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "DP_STATUS:     ", DP_STATUS_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "CONFIG1:       ", CONFIG1_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "CONFIG2:       ", CONFIG2_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "SYS_CONFIG1:   ", SYS_CONFIG1_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "SYS_CONFIG2:   ", SYS_CONFIG2_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "SYS_CONFIG3:   ", SYS_CONFIG3_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "VENDOR_CONFIG: ", VENDOR_CONFIG_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "DEV_CAPS:      ", DEV_CAPS_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "ALERT_ENABLE:  ", ALERT_ENABLE_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "DP_ALERT_EN:   ", DP_ALERT_EN_REG, 2);
	if (err)
		return err;
	err = bm92t_regs_print(s, "AUTO_NGT_FIXED:", AUTO_NGT_FIXED_REG, 5);
	if (err)
		return err;
	err = bm92t_regs_print(s, "AUTO_NGT_BATT: ", AUTO_NGT_BATT_REG, 5);
	if (err)
		return err;
	err = bm92t_regs_print(s, "CURRENT_PDO:   ", CURRENT_PDO_REG, 5);
	if (err)
		return err;
	err = bm92t_regs_print(s, "CURRENT_RDO:   ", CURRENT_RDO_REG, 5);
	if (err)
		return err;

	return 0;
}

static int bm92t_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, bm92t_regs_show, inode->i_private);
}

static const struct file_operations bm92t_regs_fops = {
	.open = bm92t_regs_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int bm92t_state_show(struct seq_file *s, void *data)
{
	struct bm92t_info *info = (struct bm92t_info *)(s->private);

	seq_printf(s, "%s\n", states[READ_ONCE(info->state)]);
	return 0;
}

static int bm92t_state_open(struct inode *inode, struct file *file)
{
	return single_open(file, bm92t_state_show, inode->i_private);
}

static const struct file_operations bm92t_state_fops = {
	.open = bm92t_state_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int bm92t_fw_info_show(struct seq_file *s, void *data)
{
	struct bm92t_info *info = (struct bm92t_info *)(s->private);

	seq_printf(s, "fw_type: 0x%02X, fw_revision: 0x%02X\n",
		   info->fw_type, info->fw_revision);
	return 0;
}

static int bm92t_fw_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, bm92t_fw_info_show, inode->i_private);
}

static const struct file_operations bm92t_fwinfo_fops = {
	.open = bm92t_fw_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t bm92t_led_write(struct file *file,
			       const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct bm92t_info *info = (struct bm92t_info *)(file->private_data);
	unsigned int duty, time_on, time_off, fade;
	char buf[32];
	int ret;

	count = min_t(size_t, count, (sizeof(buf) - 1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;

	ret = sscanf(buf, "%i %i %i %i",
		     &duty, &time_on, &time_off, &fade);

	if (ret == 4) {
		int state = READ_ONCE(info->state);

		if (state == VDM_ACCEPT_ND_ENABLE_USBHUB_REPLY ||
		    state == NINTENDO_CONFIG_HANDLED) {
			bm92t_state_machine(info, VDM_ND_CUSTOM_CMD_SENT);
			bm92t_usbhub_led_cfg(info, duty, time_on, time_off, fade);
		} else {
			dev_err(&info->i2c_client->dev,
				"Led is not supported\n");
		}
	} else {
		dev_err(&info->i2c_client->dev,
			"Led syntax is: duty time_on time_off fade\n");
		return -EINVAL;
	}

	return count;
}

static const struct file_operations bm92t_led_fops = {
	.open = simple_open,
	.write = bm92t_led_write,
};

static ssize_t bm92t_cmd_write(struct file *file,
			       const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct bm92t_info *info = (struct bm92t_info *)(file->private_data);
	unsigned int val;
	char buf[8];
	int ret;

	count = min_t(size_t, count, (sizeof(buf) - 1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;

	ret = kstrtouint(buf, 0, &val);

	if (!ret) {
		bm92t_send_cmd(info, val);
	} else {
		dev_err(&info->i2c_client->dev, "Cmd syntax is: cmd\n");
		return -EINVAL;
	}

	return count;
}

static const struct file_operations bm92t_cmd_fops = {
	.open = simple_open,
	.write = bm92t_cmd_write,
};

static ssize_t bm92t_usbhub_dp_sleep_write(struct file *file,
					   const char __user *userbuf, size_t count, loff_t *ppos)
{
	struct bm92t_info *info = (struct bm92t_info *)(file->private_data);
	unsigned int val;
	char buf[8];
	int ret;

	count = min_t(size_t, count, (sizeof(buf) - 1));
	if (copy_from_user(buf, userbuf, count))
		return -EFAULT;

	buf[count] = 0;

	ret = kstrtouint(buf, 0, &val);

	if (!ret) {
		bm92t_usbhub_dp_sleep(info, val ? true : false);
	} else {
		dev_err(&info->i2c_client->dev, "Syntax is: 0 or number\n");
		return -EINVAL;
	}

	return count;
}

static const struct file_operations bm92t_usbhub_dp_sleep_fops = {
	.open = simple_open,
	.write = bm92t_usbhub_dp_sleep_write,
};

static void bm92t_debug_init(struct bm92t_info *info)
{
	struct dentry *root = debugfs_create_dir("bm92txx", NULL);

	info->debugfs_root = root;

	debugfs_create_file("regs", 0444, root, info, &bm92t_regs_fops);
	debugfs_create_file("state", 0444, root, info, &bm92t_state_fops);
	debugfs_create_file("fw_info", 0444, root, info, &bm92t_fwinfo_fops);
	debugfs_create_file("led", 0200, root, info, &bm92t_led_fops);
	debugfs_create_file("cmd", 0200, root, info, &bm92t_cmd_fops);
	debugfs_create_file("sleep", 0200, root, info,
			    &bm92t_usbhub_dp_sleep_fops);
}
#endif

static const struct of_device_id bm92t_of_match[] = {
	{ .compatible = "rohm,bm92t", },
	{ },
};

MODULE_DEVICE_TABLE(of, bm92t_of_match);

static struct bm92t_platform_data *bm92t_parse_dt(struct device *dev)
{
	struct device_node *np = dev->of_node;
	struct bm92t_platform_data *pdata;
	int ret = 0;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->dp_disable = of_property_read_bool(np, "rohm,dp-disable");
	pdata->dp_alerts_enable = of_property_read_bool(np,
							"rohm,dp-alerts-enable");
	pdata->dp_signal_toggle_on_resume = of_property_read_bool(np,
								  "rohm,dp-signal-toggle-on-resume");
	ret = of_property_read_u32(np, "rohm,dp-lanes", &pdata->dp_lanes);
	if (ret)
		pdata->dp_lanes = 2;

	if (pdata->dp_disable) {
		pdata->dp_alerts_enable = false;
		pdata->dp_signal_toggle_on_resume = false;
		dev_info(dev, "DP handling disabled\n");
	}

	pdata->led_static_on_suspend = of_property_read_bool(np,
							     "rohm,led-static-on-suspend");
	pdata->dock_power_limit_disable = of_property_read_bool(np,
								"rohm,dock-power-limit-disable");

	ret = of_property_read_u32(np, "rohm,pd-5v-current-limit-ma",
				   &pdata->pd_5v_current_limit);
	if (ret)
		pdata->pd_5v_current_limit = PD_05V_CHARGING_CURRENT_LIMIT_MA;

	ret = of_property_read_u32(np, "rohm,pd-9v-current-limit-ma",
				   &pdata->pd_9v_current_limit);
	if (ret)
		pdata->pd_9v_current_limit = PD_09V_CHARGING_CURRENT_LIMIT_MA;

	ret = of_property_read_u32(np, "rohm,pd-12v-current-limit-ma",
				   &pdata->pd_12v_current_limit);
	if (ret)
		pdata->pd_12v_current_limit = PD_12V_CHARGING_CURRENT_LIMIT_MA;

	ret = of_property_read_u32(np, "rohm,pd-15v-current-limit-ma",
				   &pdata->pd_15v_current_limit);
	if (ret)
		pdata->pd_15v_current_limit = PD_15V_CHARGING_CURRENT_LIMIT_MA;

	return pdata;
}

static int bm92t_probe(struct i2c_client *client)
{
	struct bm92t_info *info;
	struct regulator *batt_chg_reg;
	struct regulator *vbus_reg, *vbus_src_reg;
	u16 reg_value;
	int err;

	/* Get Battery Charger and VBUS regulators */
	batt_chg_reg = devm_regulator_get(&client->dev, "pd_bat_chg");
	if (IS_ERR(batt_chg_reg)) {
		err = PTR_ERR(batt_chg_reg);
		if (err == -EPROBE_DEFER)
			return err;

		dev_err(&client->dev, "pd_bat_chg reg not registered: %d\n",
			err);
		batt_chg_reg = NULL;
	}

	vbus_reg = devm_regulator_get(&client->dev, "vbus");
	if (IS_ERR(vbus_reg)) {
		err = PTR_ERR(vbus_reg);
		if (err == -EPROBE_DEFER)
			return err;

		dev_err(&client->dev, "vbus reg not registered: %d\n", err);
		vbus_reg = NULL;
	}

	vbus_src_reg = devm_regulator_get(&client->dev, "vbus-source");
	if (IS_ERR(vbus_src_reg)) {
		err = PTR_ERR(vbus_src_reg);
		if (err == -EPROBE_DEFER)
			return err;

		dev_info(&client->dev, "no vbus source regulator provided\n");
		vbus_src_reg = NULL;
	}

	info = devm_kzalloc(&client->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	if (client->dev.of_node) {
		info->pdata = bm92t_parse_dt(&client->dev);
		if (IS_ERR(info->pdata))
			return PTR_ERR(info->pdata);
	} else {
		info->pdata = client->dev.platform_data;
		if (!info->pdata) {
			dev_err(&client->dev, "no platform data provided\n");
			return -EINVAL;
		}
	}

	/* Set VCONN pin config */
	info->vconn_en_gpio = devm_gpiod_get_optional(&client->dev,
						      "rohm,vconn-en", GPIOD_OUT_LOW);
	if (IS_ERR(info->vconn_en_gpio))
		return dev_err_probe(&client->dev, PTR_ERR(info->vconn_en_gpio),
				     "Failed to request gpio pd-vconn-en\n");

	info->role_sw = fwnode_usb_role_switch_get(dev_fwnode(&client->dev));
	if (IS_ERR_OR_NULL(info->role_sw))
		return dev_err_probe(&client->dev,
				     info->role_sw ? PTR_ERR(info->role_sw) : -ENODEV,
				     "no USB role switch\n");

	i2c_set_clientdata(client, info);

	info->i2c_client = client;
	info->batt_chg_reg = batt_chg_reg;
	info->vbus_src_reg = vbus_src_reg;
	info->vbus_reg = vbus_reg;

	/* Initialized state */
	info->state = INIT_STATE;
	info->first_init = true;
	info->usbhub_retries = BM92T_USBHUB_RETRIES;

	/* Allocate extcon */
	info->edev = devm_extcon_dev_allocate(&client->dev, bm92t_extcon_cable);
	if (IS_ERR(info->edev)) {
		err = PTR_ERR(info->edev);
		goto err_role_sw;
	}

	/* Register extcon */
	err = devm_extcon_dev_register(&client->dev, info->edev);
	if (err < 0) {
		dev_err(&client->dev, "Cannot register extcon device\n");
		goto err_role_sw;
	}

	err = bm92t_read_u16(info, FW_TYPE_REG, &reg_value);
	if (err)
		goto err_role_sw;
	info->fw_type = reg_value;

	err = bm92t_read_u16(info, FW_REVISION_REG, &reg_value);
	if (err)
		goto err_role_sw;
	info->fw_revision = reg_value;

	dev_info(&client->dev, "fw_type: 0x%02X, fw_rev: 0x%02X\n",
		 info->fw_type, info->fw_revision);

	if (info->fw_revision <= 0x644) {
		dev_err(&client->dev, "fw revision not supported\n");
		err = -EINVAL;
		goto err_role_sw;
	}

	/* Create workqueue */
	info->event_wq = create_singlethread_workqueue("bm92t-event-queue");
	if (!info->event_wq) {
		err = -ENOMEM;
		goto err_role_sw;
	}

	/* Disable Source mode at boot */
	bm92t_set_source_mode(info, SPDSRC12_OFF);

	INIT_WORK(&info->work, bm92t_event_handler);
	INIT_DELAYED_WORK(&info->oneshot_work,
			  bm92t_extcon_cable_set_init_state);

	INIT_DELAYED_WORK(&info->power_work, bm92t_power_work);
	INIT_DELAYED_WORK(&info->dp_rearm_work, bm92t_dp_rearm_work_fn);

	irq_set_status_flags(client->irq, IRQ_NOAUTOEN);
	err = request_irq(client->irq, bm92t_interrupt_handler, 0,
			  "bm92t", info);
	if (err) {
		dev_err(&client->dev, "Request irq failed\n");
		goto err_wq;
	}

#ifdef CONFIG_DEBUG_FS
	bm92t_debug_init(info);
#endif

	/* Enable VBUS source supply if available */
	if (info->vbus_src_reg) {
		err = regulator_enable(info->vbus_src_reg);
		if (err)
			dev_err(&client->dev,
				"failed to enable VBUS source: %d\n", err);
	}

	/* Enable VCONN */
	gpiod_set_value_cansleep(info->vconn_en_gpio, 1);

	schedule_delayed_work(&info->oneshot_work, msecs_to_jiffies(100));

	dev_info(&client->dev, "init done\n");

	return 0;

err_wq:
	destroy_workqueue(info->event_wq);
err_role_sw:
	usb_role_switch_put(info->role_sw);

	return err;
}

static int bm92t_pm_suspend(struct device *dev)
{
	struct bm92t_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->i2c_client;

	if (!info->vbus_suspended &&
	    !bm92t_get_vbus_enabled(info)) {
		gpiod_set_value_cansleep(info->vconn_en_gpio, 0);

		if (info->vbus_src_reg)
			regulator_disable(info->vbus_src_reg);

		info->vbus_suspended = true;
	}

	/* Dim or breathing Dock LED */
	if (info->pdata->led_static_on_suspend)
		bm92t_usbhub_led_cfg_wait(info, 16, 0, 0, 128);
	else
		bm92t_usbhub_led_cfg_wait(info, 32, 1, 255, 255);

	disable_irq(client->irq);

	return 0;
}

static int bm92t_pm_resume(struct device *dev)
{
	struct bm92t_info *info = dev_get_drvdata(dev);
	struct i2c_client *client = info->i2c_client;
	bool enable_led = READ_ONCE(info->state) == NINTENDO_CONFIG_HANDLED;
	int err;

	if (info->vbus_suspended) {
		if (info->vbus_src_reg) {
			err = regulator_enable(info->vbus_src_reg);
			if (err)
				dev_err(dev, "failed to enable VBUS source: %d\n",
					err);
		}

		gpiod_set_value_cansleep(info->vconn_en_gpio, 1);

		info->vbus_suspended = false;
	}

	enable_irq(client->irq);

	/*
	 * Toggle DP signal
	 * Do a toggle on resume instead of disable in suspend
	 * and enable in resume, because this also disables the
	 * led effects.
	 */
	if (info->pdata->dp_signal_toggle_on_resume) {
		bm92t_usbhub_dp_sleep(info, true);
		bm92t_usbhub_dp_sleep(info, false);
	}

	/* Set Dock LED to ON state */
	if (enable_led)
		bm92t_usbhub_led_cfg_wait(info, 128, 0, 0, 64);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(bm92t_pm_ops, bm92t_pm_suspend,
				bm92t_pm_resume);

static const struct i2c_device_id bm92t_id[] = {
	{ "bm92t" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bm92t_id);

static struct i2c_driver bm92t_i2c_driver = {
	.driver = {
		.name = "bm92t",
		.of_match_table = bm92t_of_match,
		.pm = pm_sleep_ptr(&bm92t_pm_ops),
	},
	.id_table = bm92t_id,
	.probe = bm92t_probe,
	.remove = bm92t_remove,
	.shutdown = bm92t_shutdown,
};
module_i2c_driver(bm92t_i2c_driver);

MODULE_AUTHOR("CTCaer <ctcaer@gmail.com>");
MODULE_DESCRIPTION("BM92TXX USB PD/DP management IC driver");
MODULE_LICENSE("GPL");
