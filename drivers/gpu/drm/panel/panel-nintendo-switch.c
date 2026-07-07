// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2018 SwtcR <swtcr0@gmail.com>
 *
 * Based on Sharp ls043t1le01 panel driver by Werner Johansson <werner.johansson@sonymobile.com>
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

struct switch_panel_variant {
	u16 id;
	const char *name;
	void (*init)(struct mipi_dsi_multi_context *ctx);
	void (*deinit)(struct mipi_dsi_multi_context *ctx);
};

struct switch_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;

	struct regulator_bulk_data supplies[3];
	bool supply_on[3];
	struct gpio_desc *reset_gpio;

	const struct switch_panel_variant *variant;
};

static inline struct switch_panel *to_switch_panel(struct drm_panel *panel)
{
	return container_of(panel, struct switch_panel, base);
}

static void jdi_init(struct mipi_dsi_multi_context *ctx)
{
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0xff, 0x83, 0x94);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xbd, 0x00);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xd8,
				     0xaa, 0xaa, 0xaa, 0xeb, 0xaa, 0xaa,
				     0xaa, 0xaa, 0xaa, 0xeb, 0xaa, 0xaa,
				     0xaa, 0xaa, 0xaa, 0xeb, 0xaa, 0xaa,
				     0xaa, 0xaa, 0xaa, 0xeb, 0xaa, 0xaa);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xbd, 0x01);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xd8,
				     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				     0xff, 0xff, 0xff, 0xff, 0xff, 0xff);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xbd, 0x02);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xd8,
				     0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
				     0xff, 0xff, 0xff, 0xff, 0xff, 0xff);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xbd, 0x00);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xd9, 0x06);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0x00, 0x00, 0x00);
}

static void jdi_deinit(struct mipi_dsi_multi_context *ctx)
{
	mipi_dsi_msleep(ctx, 50);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0xff, 0x83, 0x94);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xd5,
				     0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19,
				     0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19,
				     0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19,
				     0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb1,
				     0x41, 0x0f, 0x4f, 0x33, 0xa4, 0x79, 0xf1, 0x81,
				     0x2d, 0x00);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0x00, 0x00, 0x00);
}

static void innolux_init(struct mipi_dsi_multi_context *ctx)
{
	mipi_dsi_msleep(ctx, 180);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0xff, 0x83, 0x94);
	mipi_dsi_msleep(ctx, 5);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb1,
				     0x48, 0x15, 0x75, 0x09, 0x32, 0x14);
}

static void auo_init(struct mipi_dsi_multi_context *ctx)
{
	mipi_dsi_msleep(ctx, 180);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb9, 0xff, 0x83, 0x94);
	mipi_dsi_msleep(ctx, 5);
	mipi_dsi_dcs_write_seq_multi(ctx, 0xb1,
				     0x48, 0x11, 0x71, 0x09, 0x32, 0x14);
}

static void auo_deinit(struct mipi_dsi_multi_context *ctx)
{
	mipi_dsi_msleep(ctx, 100);
}

/* id is ID3 << 8 | ID1 of the DCS display ID, ID2 is not compared */
static const struct switch_panel_variant switch_panel_variants[] = {
	{ 0x0910, "JDI LAM062M109A", jdi_init, jdi_deinit },
	{ 0x2610, "JDI LPM062M326A", jdi_init, jdi_deinit },
	{ 0x0f20, "InnoLux P062CCA", innolux_init, NULL },
	{ 0x0f30, "AUO A062TAN", auo_init, auo_deinit },
};

static int switch_panel_identify(struct switch_panel *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	u8 id[3];
	unsigned int i;
	u16 model;
	int ret;

	ret = mipi_dsi_set_maximum_return_packet_size(ctx->dsi, sizeof(id));
	if (ret < 0)
		return ret;

	for (i = 0; i < 3; i++) {
		ret = mipi_dsi_dcs_read(ctx->dsi, MIPI_DCS_GET_DISPLAY_ID, id,
					sizeof(id));
		if (ret == sizeof(id))
			break;
		usleep_range(5000, 6000);
	}
	if (ret != sizeof(id)) {
		dev_err(dev, "failed to read display ID: %d\n", ret);
		return ret < 0 ? ret : -EIO;
	}

	model = id[2] << 8 | id[0];
	for (i = 0; i < ARRAY_SIZE(switch_panel_variants); i++) {
		if (switch_panel_variants[i].id == model) {
			ctx->variant = &switch_panel_variants[i];
			dev_dbg(dev, "%s, display ID %02x %02x %02x\n",
				ctx->variant->name, id[0], id[1], id[2]);
			return 0;
		}
	}

	dev_err(dev, "unsupported display ID %02x %02x %02x\n",
		id[0], id[1], id[2]);
	return -ENODEV;
}

static int switch_panel_supplies_on(struct switch_panel *ctx)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ctx->supplies); i++) {
		if (ctx->supply_on[i])
			continue;

		ret = regulator_enable(ctx->supplies[i].consumer);
		if (ret)
			return ret;
		ctx->supply_on[i] = true;
		msleep(20);
	}

	return 0;
}

static void switch_panel_supplies_off(struct switch_panel *ctx)
{
	int i, ret;

	for (i = ARRAY_SIZE(ctx->supplies) - 1; i >= 0; i--) {
		if (!ctx->supply_on[i])
			continue;

		ret = regulator_disable(ctx->supplies[i].consumer);
		if (ret) {
			dev_err(ctx->base.dev, "failed to disable %s: %d\n",
				ctx->supplies[i].supply, ret);
			return;
		}
		ctx->supply_on[i] = false;
		msleep(20);
	}
}

static void switch_panel_power_off(struct switch_panel *ctx)
{
	int ret;

	ret = gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	if (ret)
		dev_err(ctx->base.dev, "failed to assert reset: %d\n", ret);

	msleep(20);
	switch_panel_supplies_off(ctx);
}

static int switch_panel_unprepare(struct drm_panel *panel)
{
	struct switch_panel *ctx = to_switch_panel(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi_ctx);
	if (ctx->variant->deinit)
		ctx->variant->deinit(&dsi_ctx);
	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 50);

	switch_panel_power_off(ctx);

	return 0;
}

static int switch_panel_prepare(struct drm_panel *panel)
{
	struct switch_panel *ctx = to_switch_panel(panel);
	struct mipi_dsi_multi_context dsi_ctx = { .dsi = ctx->dsi };
	int ret;

	ret = switch_panel_supplies_on(ctx);
	if (ret)
		goto power_off;

	ret = gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	if (ret)
		goto power_off;
	msleep(20);
	ret = gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	if (ret)
		goto power_off;
	msleep(120);

	ret = switch_panel_identify(ctx);
	if (ret)
		goto power_off;

	msleep(20);
	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi_ctx);
	ctx->variant->init(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 360);
	mipi_dsi_dcs_set_column_address_multi(&dsi_ctx, 0, 720 - 1);
	mipi_dsi_dcs_set_page_address_multi(&dsi_ctx, 0, 1280 - 1);
	mipi_dsi_dcs_set_tear_on_multi(&dsi_ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_dcs_set_pixel_format_multi(&dsi_ctx, MIPI_DCS_PIXEL_FMT_24BIT);
	mipi_dsi_dcs_set_display_on_multi(&dsi_ctx);
	mipi_dsi_msleep(&dsi_ctx, 40);
	ret = dsi_ctx.accum_err;
	if (ret)
		goto power_off;

	return 0;

power_off:
	dev_err(panel->dev, "failed to prepare: %d\n", ret);
	switch_panel_power_off(ctx);
	return ret;
}

static int switch_panel_enable(struct drm_panel *panel)
{
	/* Tegra DSI starts video and enables the panel after a failed prepare */
	return panel->prepared ? 0 : -EIO;
}

static const struct drm_display_mode switch_panel_mode = {
	.clock = 78000,
	.hdisplay = 720,
	.hsync_start = 720 + 136,
	.hsync_end = 720 + 136 + 72,
	.htotal = 720 + 136 + 72 + 72,
	.vdisplay = 1280,
	.vsync_start = 1280 + 10,
	.vsync_end = 1280 + 10 + 1,
	.vtotal = 1280 + 10 + 1 + 9,
	.width_mm = 77,
	.height_mm = 137,
};

static int switch_panel_get_modes(struct drm_panel *panel,
				  struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector,
						    &switch_panel_mode);
}

static const struct drm_panel_funcs switch_panel_funcs = {
	.unprepare = switch_panel_unprepare,
	.prepare = switch_panel_prepare,
	.enable = switch_panel_enable,
	.get_modes = switch_panel_get_modes,
};

static void switch_panel_release(void *data)
{
	switch_panel_power_off(data);
}

static int switch_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct switch_panel *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct switch_panel, base,
				   &switch_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dsi = dsi;

	ctx->supplies[0].supply = "vddi";
	ctx->supplies[1].supply = "avdd";
	ctx->supplies[2].supply = "avee";
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ctx->supplies),
				      ctx->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get supplies\n");

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset GPIO\n");

	ret = drm_panel_of_backlight(&ctx->base);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get backlight\n");

	/* runs after the DSI detach, which powers an active panel down */
	ret = devm_add_action(dev, switch_panel_release, ctx);
	if (ret)
		return ret;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM |
			  MIPI_DSI_CLOCK_NON_CONTINUOUS |
			  MIPI_DSI_MODE_NO_EOT_PACKET;

	ret = devm_drm_panel_add(dev, &ctx->base);
	if (ret)
		return ret;

	return devm_mipi_dsi_attach(dev, dsi);
}

static const struct of_device_id switch_panel_of_match[] = {
	{ .compatible = "nintendo,switch-panel" },
	{ }
};
MODULE_DEVICE_TABLE(of, switch_panel_of_match);

static struct mipi_dsi_driver switch_panel_driver = {
	.driver = {
		.name = "panel-nintendo-switch",
		.of_match_table = switch_panel_of_match,
	},
	.probe = switch_panel_probe,
};
module_mipi_dsi_driver(switch_panel_driver);

MODULE_AUTHOR("SwtcR <swtcr0@gmail.com>");
MODULE_DESCRIPTION("Nintendo Switch 6.2\" 720x1280 DSI panel driver");
MODULE_LICENSE("GPL");
