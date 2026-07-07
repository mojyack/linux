// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018 SwtcR <swtcr0@gmail.com>
 *
 * Based on Sharp ls043t1le01 panel driver by Werner Johansson <werner.johansson@sonymobile.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_crtc.h>
#include <drm/drm_device.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

struct init_cmd {
	u8 cmd;
	int length;
	u8 data[0x40];
};

struct jdi_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	struct regulator *supply1;
	struct regulator *supply2;
	struct gpio_desc *reset_gpio;

	const struct init_cmd *suspend_cmds;

	const struct drm_display_mode *mode;
};

/* cmd 0xff entries are delays: length is the duration in ms */
#define CMD_DELAY_MS 0xff

static const struct init_cmd init_cmds_0x10[] = {
	{ 0xb9, 3,    { 0xff, 0x83, 0x94 }},
	{ 0xbd, 1,    { 0x00 }},
	{ 0xd8, 0x18, { 0xaa, 0xaa, 0xaa, 0xeb, 0xaa, 0xaa,
			0xaa, 0xaa, 0xaa, 0xeb, 0xaa, 0xaa,
			0xaa, 0xaa, 0xaa, 0xeb, 0xaa, 0xaa,
			0xaa, 0xaa, 0xaa, 0xeb, 0xaa, 0xaa }},
	{ 0xbd, 1,    { 0x01 }},
	{ 0xd8, 0x26, { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
			0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
			0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
			0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
			0xff, 0xff, 0xff, 0xff, 0xff, 0xff }},
	{ 0xbd, 1,    { 0x02 }},
	{ 0xd8, 0xe,  { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
			0xff, 0xff, 0xff, 0xff, 0xff, 0xff }},
	{ 0xbd, 1,    { 0x00 }},
	{ 0xd9, 1,    { 0x06 }},
	{ 0xb9, 3,    { 0x00, 0x00, 0x00 }},
	{ 0x00, -1,   { 0x00, }},
};

/* InnoLux P062CCA-AZ1 */
static const struct init_cmd init_cmds_0x0f20[] = {
	{ CMD_DELAY_MS, 180 },
	{ 0xb9, 3,    { 0xff, 0x83, 0x94 }},
	{ CMD_DELAY_MS, 5 },
	{ 0xb1, 6,    { 0x48, 0x15, 0x75, 0x09, 0x32, 0x14 }},
	{ 0x00, -1,   { 0x00, }},
};

/* AUO A062TAN01 */
static const struct init_cmd init_cmds_0x0f30[] = {
	{ CMD_DELAY_MS, 180 },
	{ 0xb9, 3,    { 0xff, 0x83, 0x94 }},
	{ CMD_DELAY_MS, 5 },
	{ 0xb1, 6,    { 0x48, 0x11, 0x71, 0x09, 0x32, 0x14 }},
	{ 0x00, -1,   { 0x00, }},
};

static const struct init_cmd suspend_cmds_0x10[] = {
	{ CMD_DELAY_MS, 50 },
	{ 0xb9, 3,    { 0xff, 0x83, 0x94 }},
	{ 0xd5, 32,   { 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19,
			0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19,
			0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19,
			0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19 }},
	{ 0xb1, 10,   { 0x41, 0x0f, 0x4f, 0x33, 0xa4, 0x79, 0xf1, 0x81,
			0x2d, 0x00 }},
	{ 0xb9, 3,    { 0x00, 0x00, 0x00 }},
	{ 0x00, -1,   { 0x00, }},
};

static const struct init_cmd suspend_cmds_0x0f30[] = {
	{ CMD_DELAY_MS, 100 },
	{ 0x00, -1,   { 0x00, }},
};

static inline struct jdi_panel *to_jdi_panel(struct drm_panel *panel)
{
	return container_of(panel, struct jdi_panel, base);
}

static int jdi_panel_write_cmds(struct mipi_dsi_device *dsi,
				const struct init_cmd *cmds)
{
	int ret;

	while (cmds && cmds->length != -1) {
		if (cmds->cmd == CMD_DELAY_MS) {
			msleep(cmds->length);
		} else {
			ret = mipi_dsi_dcs_write(dsi, cmds->cmd, cmds->data,
						 cmds->length);
			if (ret < 0)
				return ret;
		}
		cmds++;
	}

	return 0;
}

static int jdi_panel_init(struct jdi_panel *jdi)
{
	struct mipi_dsi_device *dsi = jdi->dsi;
	int ret;
	u8 display_id[3] = {0};
	u16 panel_id;
	const struct init_cmd *init_cmds = NULL;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_set_maximum_return_packet_size(dsi, 3);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_read(dsi, MIPI_DCS_GET_DISPLAY_ID, display_id,
				sizeof(display_id));
	if (ret < 0) {
		dev_err(&dsi->dev, "failed to read panel ID: %d\n", ret);
	} else {
		dev_info(&dsi->dev, "display ID[%d]: %02x %02x %02x\n",
			 ret, display_id[0], display_id[1], display_id[2]);
	}

	/* ID1 is vendor, ID3 family; all JDI models share one sequence. */
	panel_id = display_id[0] == 0x10 ?
		0x10 : (display_id[2] << 8) | display_id[0];
	jdi->suspend_cmds = NULL;

	switch (panel_id) {
	case 0x10: /* JDI LPM062M326A / LAM062M109A */
		dev_info(&dsi->dev, "using JDI init sequence\n");
		init_cmds = init_cmds_0x10;
		jdi->suspend_cmds = suspend_cmds_0x10;
		break;
	case 0x0f20: /* InnoLux P062CCA-AZ1 */
		dev_info(&dsi->dev, "using InnoLux init sequence\n");
		init_cmds = init_cmds_0x0f20;
		break;
	case 0x0f30: /* AUO A062TAN01 */
		dev_info(&dsi->dev, "using AUO init sequence\n");
		init_cmds = init_cmds_0x0f30;
		jdi->suspend_cmds = suspend_cmds_0x0f30;
		break;
	default:
		dev_info(&dsi->dev, "unknown display %04x, no extra init\n",
			 panel_id);
		break;
	}

	msleep(20);

	ret = mipi_dsi_dcs_exit_sleep_mode(dsi);
	if (ret < 0)
		return ret;

	ret = jdi_panel_write_cmds(dsi, init_cmds);
	if (ret < 0)
		return ret;

	msleep(360);

	ret = mipi_dsi_dcs_set_column_address(dsi, 0,
					      jdi->mode->hdisplay - 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_set_page_address(dsi, 0,
					    jdi->mode->vdisplay - 1);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_set_tear_on(dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_set_pixel_format(dsi, MIPI_DCS_PIXEL_FMT_24BIT);
	if (ret < 0)
		return ret;

	return 0;
}

static int jdi_panel_on(struct jdi_panel *jdi)
{
	struct mipi_dsi_device *dsi = jdi->dsi;
	int ret;

	dsi->mode_flags |= MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_on(dsi);
	if (ret < 0)
		return ret;

	msleep(40);

	return 0;
}

static int jdi_panel_off(struct jdi_panel *jdi)
{
	struct mipi_dsi_device *dsi = jdi->dsi;
	int ret;

	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;

	ret = mipi_dsi_dcs_set_display_off(dsi);
	if (ret < 0)
		return ret;

	ret = jdi_panel_write_cmds(dsi, jdi->suspend_cmds);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_dcs_enter_sleep_mode(dsi);
	if (ret < 0)
		return ret;

	msleep(50);

	return 0;
}

static int jdi_panel_unprepare(struct drm_panel *panel)
{
	struct jdi_panel *jdi = to_jdi_panel(panel);
	int ret;

	ret = jdi_panel_off(jdi);
	if (ret < 0) {
		dev_err(panel->dev, "failed to set panel off: %d\n", ret);
		return ret;
	}

	if (jdi->reset_gpio)
		gpiod_set_value(jdi->reset_gpio, 0);

	msleep(20);
	regulator_disable(jdi->supply2);
	msleep(20);
	regulator_disable(jdi->supply1);

	return 0;
}

static int jdi_panel_prepare(struct drm_panel *panel)
{
	struct jdi_panel *jdi = to_jdi_panel(panel);
	int ret;

	ret = regulator_enable(jdi->supply1);
	if (ret < 0)
		return ret;
	msleep(20);
	ret = regulator_enable(jdi->supply2);
	if (ret < 0)
		goto poweroff1;
	msleep(20);

	if (jdi->reset_gpio) {
		gpiod_set_value(jdi->reset_gpio, 0);
		msleep(20);
		gpiod_set_value(jdi->reset_gpio, 1);
		msleep(120);
	}

	ret = jdi_panel_init(jdi);
	if (ret < 0) {
		dev_err(panel->dev, "failed to init panel: %d\n", ret);
		goto reset;
	}

	ret = jdi_panel_on(jdi);
	if (ret < 0) {
		dev_err(panel->dev, "failed to set panel on: %d\n", ret);
		goto reset;
	}

	return 0;

reset:
	if (jdi->reset_gpio)
		gpiod_set_value(jdi->reset_gpio, 0);
	regulator_disable(jdi->supply2);
poweroff1:
	regulator_disable(jdi->supply1);
	return ret;
}

static const struct drm_display_mode default_mode = {
	.clock = 78000,
	.hdisplay = 720,
	.hsync_start = 720 + 136,
	.hsync_end = 720 + 136 + 72,
	.htotal = 720 + 136 + 72 + 72,
	.vdisplay = 1280,
	.vsync_start = 1280 + 10,
	.vsync_end = 1280 + 10 + 2,
	.vtotal = 1280 + 10 + 1 + 9,
};

static int jdi_panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(connector->dev, &default_mode);
	if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
		return -ENOMEM;
	}

	drm_mode_set_name(mode);

	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = 77;
	connector->display_info.height_mm = 137;

	return 1;
}

static const struct drm_panel_funcs jdi_panel_funcs = {
	.unprepare = jdi_panel_unprepare,
	.prepare = jdi_panel_prepare,
	.get_modes = jdi_panel_get_modes,
};

static int jdi_panel_add(struct jdi_panel *jdi)
{
	struct device *dev = &jdi->dsi->dev;
	int ret;

	jdi->mode = &default_mode;

	jdi->supply1 = devm_regulator_get(dev, "vdd1");
	if (IS_ERR(jdi->supply1))
		return PTR_ERR(jdi->supply1);

	jdi->supply2 = devm_regulator_get(dev, "vdd2");
	if (IS_ERR(jdi->supply2))
		return PTR_ERR(jdi->supply2);

	jdi->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(jdi->reset_gpio)) {
		dev_err(dev, "cannot get reset-gpios %ld\n",
			PTR_ERR(jdi->reset_gpio));
		jdi->reset_gpio = NULL;
	} else {
		gpiod_set_value(jdi->reset_gpio, 0);
	}

	ret = drm_panel_of_backlight(&jdi->base);
	if (ret)
		return ret;

	drm_panel_add(&jdi->base);

	return 0;
}

static void jdi_panel_del(struct jdi_panel *jdi)
{
	if (jdi->base.dev)
		drm_panel_remove(&jdi->base);
}

static int jdi_panel_probe(struct mipi_dsi_device *dsi)
{
	struct jdi_panel *jdi;
	int ret;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO |
			MIPI_DSI_CLOCK_NON_CONTINUOUS |
			MIPI_DSI_MODE_NO_EOT_PACKET;

	jdi = devm_drm_panel_alloc(&dsi->dev, struct jdi_panel, base,
				   &jdi_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(jdi))
		return PTR_ERR(jdi);

	mipi_dsi_set_drvdata(dsi, jdi);

	jdi->dsi = dsi;

	ret = jdi_panel_add(jdi);
	if (ret < 0)
		return ret;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) {
		jdi_panel_del(jdi);
		return ret;
	}

	return 0;
}

static void jdi_panel_remove(struct mipi_dsi_device *dsi)
{
	struct jdi_panel *jdi = mipi_dsi_get_drvdata(dsi);
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n", ret);

	jdi_panel_del(jdi);
}

static const struct of_device_id jdi_of_match[] = {
	{ .compatible = "jdi,lpm062m326a", },
	{ }
};
MODULE_DEVICE_TABLE(of, jdi_of_match);

static struct mipi_dsi_driver jdi_panel_driver = {
	.driver = {
		.name = "panel-jdi-lpm062m326a",
		.of_match_table = jdi_of_match,
	},
	.probe = jdi_panel_probe,
	.remove = jdi_panel_remove,
};
module_mipi_dsi_driver(jdi_panel_driver);

MODULE_AUTHOR("SwtcR <swtcr0@gmail.com>");
MODULE_DESCRIPTION("JDI LPM062M326A (720x1280) panel driver");
MODULE_LICENSE("GPL");
