// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 Maxime Ripard <mripard@kernel.org>
 */

#include <kunit/test.h>

#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_client.h>
#include <drm/drm_connector.h>
#include <drm/drm_edid.h>
#include <drm/drm_drv.h>
#include <drm/drm_kunit_helpers.h>
#include <drm/drm_managed.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>

struct drm_client_modeset_test_priv {
	struct drm_device *drm;
	struct device *dev;
	struct drm_connector connector;
};

static int drm_client_modeset_connector_get_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;
	int count;

	count = drm_add_modes_noedid(connector, 1920, 1200);

	mode = drm_mode_analog_ntsc_480i(connector->dev);
	if (!mode)
		return count;

	drm_mode_probed_add(connector, mode);
	count += 1;

	mode = drm_mode_analog_pal_576i(connector->dev);
	if (!mode)
		return count;

	drm_mode_probed_add(connector, mode);
	count += 1;

	return count;
}

static const struct drm_connector_helper_funcs drm_client_modeset_connector_helper_funcs = {
	.get_modes = drm_client_modeset_connector_get_modes,
};

static const struct drm_connector_funcs drm_client_modeset_connector_funcs = {
	.atomic_destroy_state   = drm_atomic_helper_connector_destroy_state,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state
};

static int drm_client_modeset_test_init(struct kunit *test)
{
	struct drm_client_modeset_test_priv *priv;
	int ret;

	priv = kunit_kzalloc(test, sizeof(*priv), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, priv);

	test->priv = priv;

	priv->dev = drm_kunit_helper_alloc_device(test);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, priv->dev);

	priv->drm = __drm_kunit_helper_alloc_drm_device(test, priv->dev,
							sizeof(*priv->drm), 0,
							DRIVER_MODESET);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, priv->drm);

	ret = drmm_connector_init(priv->drm, &priv->connector,
				  &drm_client_modeset_connector_funcs,
				  DRM_MODE_CONNECTOR_Unknown,
				  NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);

	drm_connector_helper_add(&priv->connector, &drm_client_modeset_connector_helper_funcs);

	priv->connector.interlace_allowed = true;
	priv->connector.doublescan_allowed = true;

	return 0;
}

static void drm_test_pick_cmdline_res_1920_1080_60(struct kunit *test)
{
	struct drm_client_modeset_test_priv *priv = test->priv;
	struct drm_device *drm = priv->drm;
	struct drm_connector *connector = &priv->connector;
	struct drm_cmdline_mode *cmdline_mode = &connector->cmdline_mode;
	struct drm_display_mode *expected_mode;
	const struct drm_display_mode *mode;
	const char *cmdline = "1920x1080@60";
	int ret;

	expected_mode = drm_mode_find_dmt(priv->drm, 1920, 1080, 60, false);
	KUNIT_ASSERT_NOT_NULL(test, expected_mode);

	ret = drm_kunit_add_mode_destroy_action(test, expected_mode);
	KUNIT_ASSERT_EQ(test, ret, 0);

	KUNIT_ASSERT_TRUE(test,
			  drm_mode_parse_command_line_for_connector(cmdline,
								    connector,
								    cmdline_mode));

	mutex_lock(&drm->mode_config.mutex);
	ret = drm_helper_probe_single_connector_modes(connector, 1920, 1080);
	mutex_unlock(&drm->mode_config.mutex);
	KUNIT_ASSERT_GT(test, ret, 0);

	mode = drm_connector_pick_cmdline_mode(connector, 1920, 1080);
	KUNIT_ASSERT_NOT_NULL(test, mode);

	KUNIT_EXPECT_TRUE(test, drm_mode_equal(expected_mode, mode));
}

static void drm_test_pick_cmdline_too_large(struct kunit *test)
{
	struct drm_client_modeset_test_priv *priv = test->priv;
	struct drm_device *drm = priv->drm;
	struct drm_connector *connector = &priv->connector;
	struct drm_cmdline_mode *cmdline_mode = &connector->cmdline_mode;
	const struct drm_display_mode *mode;
	struct drm_display_mode *probed;
	const char *cmdline = "1920x1080@60";
	int ret;

	KUNIT_ASSERT_TRUE(test,
			  drm_mode_parse_command_line_for_connector(cmdline,
								    connector,
								    cmdline_mode));

	mutex_lock(&drm->mode_config.mutex);
	ret = drm_helper_probe_single_connector_modes(connector, 4096, 4096);
	mutex_unlock(&drm->mode_config.mutex);
	KUNIT_ASSERT_GT(test, ret, 0);

	mode = drm_connector_pick_cmdline_mode(connector, 1920, 1080);
	KUNIT_ASSERT_NOT_NULL(test, mode);
	KUNIT_ASSERT_TRUE(test, mode->type & DRM_MODE_TYPE_USERDEF);

	KUNIT_EXPECT_NULL(test, drm_connector_pick_cmdline_mode(connector, 1280, 720));

	/* Same again through the xres/yres match */
	list_for_each_entry(probed, &connector->modes, head)
		probed->type &= ~DRM_MODE_TYPE_USERDEF;

	mode = drm_connector_pick_cmdline_mode(connector, 1920, 1080);
	KUNIT_ASSERT_NOT_NULL(test, mode);
	KUNIT_EXPECT_EQ(test, mode->hdisplay, 1920);
	KUNIT_EXPECT_EQ(test, mode->vdisplay, 1080);

	KUNIT_EXPECT_NULL(test, drm_connector_pick_cmdline_mode(connector, 1280, 720));
}

struct drm_connector_pick_cmdline_mode_test {
	const char *cmdline;
	struct drm_display_mode *(*func)(struct drm_device *drm);
};

#define TEST_CMDLINE(_cmdline, _fn)		\
	{					\
		.cmdline = _cmdline,		\
		.func = _fn,			\
	}

static void drm_test_pick_cmdline_named(struct kunit *test)
{
	const struct drm_connector_pick_cmdline_mode_test *params = test->param_value;
	struct drm_client_modeset_test_priv *priv = test->priv;
	struct drm_device *drm = priv->drm;
	struct drm_connector *connector = &priv->connector;
	struct drm_cmdline_mode *cmdline_mode = &connector->cmdline_mode;
	const struct drm_display_mode *mode;
	struct drm_display_mode *expected_mode;
	const char *cmdline = params->cmdline;
	int ret;

	KUNIT_ASSERT_TRUE(test,
			  drm_mode_parse_command_line_for_connector(cmdline,
								    connector,
								    cmdline_mode));

	mutex_lock(&drm->mode_config.mutex);
	ret = drm_helper_probe_single_connector_modes(connector, 1920, 1080);
	mutex_unlock(&drm->mode_config.mutex);
	KUNIT_ASSERT_GT(test, ret, 0);

	mode = drm_connector_pick_cmdline_mode(connector, 1920, 1080);
	KUNIT_ASSERT_NOT_NULL(test, mode);

	expected_mode = params->func(drm);
	KUNIT_ASSERT_NOT_NULL(test, expected_mode);

	ret = drm_kunit_add_mode_destroy_action(test, expected_mode);
	KUNIT_ASSERT_EQ(test, ret, 0);

	KUNIT_EXPECT_TRUE(test, drm_mode_equal(expected_mode, mode));
}

static const
struct drm_connector_pick_cmdline_mode_test drm_connector_pick_cmdline_mode_tests[] = {
	TEST_CMDLINE("NTSC", drm_mode_analog_ntsc_480i),
	TEST_CMDLINE("NTSC-J", drm_mode_analog_ntsc_480i),
	TEST_CMDLINE("PAL", drm_mode_analog_pal_576i),
	TEST_CMDLINE("PAL-M", drm_mode_analog_ntsc_480i),
};

static void
drm_connector_pick_cmdline_mode_desc(const struct drm_connector_pick_cmdline_mode_test *t,
				     char *desc)
{
	sprintf(desc, "%s", t->cmdline);
}

KUNIT_ARRAY_PARAM(drm_connector_pick_cmdline_mode,
		  drm_connector_pick_cmdline_mode_tests,
		  drm_connector_pick_cmdline_mode_desc);

static struct kunit_case drm_test_pick_cmdline_tests[] = {
	KUNIT_CASE(drm_test_pick_cmdline_res_1920_1080_60),
	KUNIT_CASE(drm_test_pick_cmdline_too_large),
	KUNIT_CASE_PARAM(drm_test_pick_cmdline_named,
			 drm_connector_pick_cmdline_mode_gen_params),
	{}
};

static struct kunit_suite drm_test_pick_cmdline_test_suite = {
	.name = "drm_test_pick_cmdline",
	.init = drm_client_modeset_test_init,
	.test_cases = drm_test_pick_cmdline_tests
};

kunit_test_suite(drm_test_pick_cmdline_test_suite);

static int drm_client_modeset_dp_get_modes(struct drm_connector *connector)
{
	struct drm_display_mode *mode;

	mode = drm_mode_find_dmt(connector->dev, 1920, 1080, 60, false);
	if (!mode)
		return 0;

	mode->type |= DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode);

	return 1;
}

static const struct drm_connector_helper_funcs drm_client_modeset_dp_helper_funcs = {
	.get_modes = drm_client_modeset_dp_get_modes,
};

/*
 * A DP sink listing only modes larger than the client still gives the client
 * the 640x480 fail-safe mode, and the larger mode stays on the list.
 */
static void drm_test_client_fit_dp_failsafe(struct kunit *test)
{
	struct drm_client_modeset_test_priv *priv = test->priv;
	struct drm_device *drm = priv->drm;
	const struct drm_display_mode *mode;
	struct drm_connector *connector;
	bool found = false;
	int ret;

	connector = drmm_kzalloc(drm, sizeof(*connector), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, connector);

	ret = drmm_connector_init(drm, connector,
				  &drm_client_modeset_connector_funcs,
				  DRM_MODE_CONNECTOR_DisplayPort, NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);
	drm_connector_helper_add(connector, &drm_client_modeset_dp_helper_funcs);

	mutex_lock(&drm->mode_config.mutex);
	ret = drm_helper_probe_single_connector_modes(connector, 4096, 4096);
	mutex_unlock(&drm->mode_config.mutex);
	KUNIT_ASSERT_GT(test, ret, 0);

	list_for_each_entry(mode, &connector->modes, head) {
		if (mode->hdisplay == 1920 && mode->vdisplay == 1080)
			found = true;
	}
	KUNIT_EXPECT_TRUE(test, found);

	KUNIT_EXPECT_NULL(test, drm_connector_preferred_mode(connector, 1280, 720));

	mode = drm_connector_first_mode(connector, 1280, 720);
	KUNIT_ASSERT_NOT_NULL(test, mode);
	KUNIT_EXPECT_EQ(test, mode->hdisplay, 640);
	KUNIT_EXPECT_EQ(test, mode->vdisplay, 480);
}

static void drm_test_client_fit_tiled(struct kunit *test)
{
	struct drm_client_modeset_test_priv *priv = test->priv;
	struct drm_device *drm = priv->drm;
	struct drm_connector *connector = &priv->connector;
	const struct drm_display_mode *mode;
	int ret;

	mutex_lock(&drm->mode_config.mutex);
	ret = drm_helper_probe_single_connector_modes(connector, 4096, 4096);
	mutex_unlock(&drm->mode_config.mutex);
	KUNIT_ASSERT_GT(test, ret, 0);

	connector->has_tile = true;
	connector->num_h_tile = 2;
	connector->num_v_tile = 1;
	connector->tile_h_size = 1920;
	connector->tile_v_size = 1200;

	KUNIT_EXPECT_NOT_NULL(test, drm_connector_get_tiled_mode(connector, 1920, 1200));
	KUNIT_EXPECT_NULL(test, drm_connector_get_tiled_mode(connector, 1280, 720));

	mode = drm_connector_fallback_non_tiled_mode(connector, 1280, 720);
	KUNIT_ASSERT_NOT_NULL(test, mode);
	KUNIT_EXPECT_LE(test, mode->hdisplay, 1280);
	KUNIT_EXPECT_LE(test, mode->vdisplay, 720);
}

static void drm_test_client_fit_cloned(struct kunit *test)
{
	struct drm_client_modeset_test_priv *priv = test->priv;
	struct drm_device *drm = priv->drm;
	const struct drm_display_mode *modes[2] = {};
	struct drm_client_offset offsets[2] = {};
	struct drm_connector *connectors[2];
	bool enabled[2] = { true, true };
	int ret, i;

	connectors[0] = &priv->connector;
	connectors[1] = drmm_kzalloc(drm, sizeof(*connectors[1]), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, connectors[1]);

	ret = drmm_connector_init(drm, connectors[1],
				  &drm_client_modeset_connector_funcs,
				  DRM_MODE_CONNECTOR_Unknown, NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);
	drm_connector_helper_add(connectors[1], &drm_client_modeset_connector_helper_funcs);

	mutex_lock(&drm->mode_config.mutex);
	for (i = 0; i < 2; i++) {
		ret = drm_helper_probe_single_connector_modes(connectors[i], 4096, 4096);
		KUNIT_EXPECT_GT(test, ret, 0);
	}

	KUNIT_EXPECT_FALSE(test, drm_client_target_cloned(drm, connectors, 2, modes,
							  offsets, enabled, 800, 600));
	modes_destroy(drm, modes, 2);

	KUNIT_EXPECT_TRUE(test, drm_client_target_cloned(drm, connectors, 2, modes,
							 offsets, enabled, 1024, 768));
	modes_destroy(drm, modes, 2);
	mutex_unlock(&drm->mode_config.mutex);
}

struct drm_client_modeset_recording_connector {
	struct drm_connector base;
	u32 max_width;
	u32 max_height;
};

static int drm_client_modeset_record_fill_modes(struct drm_connector *connector,
						u32 max_width, u32 max_height)
{
	struct drm_client_modeset_recording_connector *rec =
		container_of(connector, typeof(*rec), base);

	rec->max_width = max_width;
	rec->max_height = max_height;

	return 0;
}

static const struct drm_connector_funcs drm_client_modeset_recording_connector_funcs = {
	.fill_modes		= drm_client_modeset_record_fill_modes,
	.atomic_destroy_state	= drm_atomic_helper_connector_destroy_state,
	.atomic_duplicate_state	= drm_atomic_helper_connector_duplicate_state
};

static int drm_client_modeset_dumb_create(struct drm_file *file_priv,
					  struct drm_device *dev,
					  struct drm_mode_create_dumb *args)
{
	return -EOPNOTSUPP;
}

static const struct drm_driver drm_client_modeset_client_driver = {
	.driver_features	= DRIVER_MODESET,
	.dumb_create		= drm_client_modeset_dumb_create,
};

/* A client smaller than the device still probes at the device's limits. */
static void drm_test_client_fit_probe(struct kunit *test)
{
	struct drm_client_modeset_test_priv *priv = test->priv;
	struct drm_client_modeset_recording_connector *rec;
	struct drm_client_dev client = {};
	struct drm_device *drm;
	int ret;

	drm = __drm_kunit_helper_alloc_drm_device_with_driver(test, priv->dev, sizeof(*drm), 0,
							      &drm_client_modeset_client_driver);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, drm);
	drm->mode_config.max_width = 4096;
	drm->mode_config.max_height = 2160;

	rec = drmm_kzalloc(drm, sizeof(*rec), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, rec);

	ret = drmm_connector_init(drm, &rec->base,
				  &drm_client_modeset_recording_connector_funcs,
				  DRM_MODE_CONNECTOR_DisplayPort, NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);

	ret = drm_client_init(drm, &client, "kunit", NULL);
	KUNIT_ASSERT_EQ(test, ret, 0);

	ret = drm_client_modeset_probe(&client, 1280, 720);
	drm_client_release(&client);
	KUNIT_ASSERT_EQ(test, ret, 0);

	KUNIT_EXPECT_EQ(test, rec->max_width, 4096);
	KUNIT_EXPECT_EQ(test, rec->max_height, 2160);
}

static struct kunit_case drm_test_client_fit_tests[] = {
	KUNIT_CASE(drm_test_client_fit_dp_failsafe),
	KUNIT_CASE(drm_test_client_fit_tiled),
	KUNIT_CASE(drm_test_client_fit_cloned),
	KUNIT_CASE(drm_test_client_fit_probe),
	{}
};

static struct kunit_suite drm_test_client_fit_test_suite = {
	.name = "drm_test_client_fit",
	.init = drm_client_modeset_test_init,
	.test_cases = drm_test_client_fit_tests
};

kunit_test_suite(drm_test_client_fit_test_suite);

/*
 * This file is included directly by drm_client_modeset.c so we can't
 * use any MODULE_* macro here.
 */
