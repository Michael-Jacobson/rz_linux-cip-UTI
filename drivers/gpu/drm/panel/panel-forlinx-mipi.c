// SPDX-License-Identifier: GPL-2.0
/*
 * Raydium RM67191 MIPI-DSI panel driver
 *
 * Copyright 2019 NXP
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

/* Panel specific color-format bits */
#define COL_FMT_16BPP 0x55
#define COL_FMT_18BPP 0x66
#define COL_FMT_24BPP 0x77


static const u32 forlinx_bus_formats[] = {
	MEDIA_BUS_FMT_RGB888_1X24,
	MEDIA_BUS_FMT_RGB666_1X18,
	MEDIA_BUS_FMT_RGB565_1X16,
};

static const u32 forlinx_bus_flags = DRM_BUS_FLAG_DE_LOW |
				 DRM_BUS_FLAG_PIXDATA_SAMPLE_POSEDGE;

struct forlinx_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;

	struct gpio_desc *enable;
	struct backlight_device *backlight;

	struct regulator_bulk_data *supplies;
	unsigned int num_supplies;

	bool prepared;
	bool enabled;

	const struct forlinx_platform_data *pdata;
};

struct forlinx_platform_data {
	int (*enable)(struct forlinx_panel *panel);
};

static const struct drm_display_mode default_mode = {
	.clock = 45000,
	.hdisplay = 1024,
	.hsync_start = 1024 + 160,     //bp
	.hsync_end = 1024 + 160 + 70,   //bp sw
	.htotal = 1024 + 160 + 70 + 160, //bp sw fp
	.vdisplay = 600,
	.vsync_start = 600 + 23,
	.vsync_end = 600 + 23 + 10,
	.vtotal = 600 + 23 + 10 + 12,
	.width_mm = 68,
	.height_mm = 121,
	.flags = DRM_MODE_FLAG_NHSYNC |
		 DRM_MODE_FLAG_NVSYNC,
};

static inline struct forlinx_panel *to_forlinx_panel(struct drm_panel *panel)
{
	return container_of(panel, struct forlinx_panel, panel);
}


static int forlinx_panel_prepare(struct drm_panel *panel)
{
	struct forlinx_panel *forlinx = to_forlinx_panel(panel);

	if (forlinx->prepared)
		return 0;

	if(forlinx->enable)
		gpiod_set_value_cansleep(forlinx->enable, 1);

	forlinx->prepared = true;

	return 0;
}

static int forlinx_panel_unprepare(struct drm_panel *panel)
{
	struct forlinx_panel *forlinx = to_forlinx_panel(panel);

	if (!forlinx->prepared)
		return 0;

	if(forlinx->enable)
		gpiod_set_value_cansleep(forlinx->enable, 0);

	forlinx->prepared = false;

	return 0;
}

static int rm67191_enable(struct forlinx_panel *panel)
{
	if (panel->enabled)
		return 0;

	if(panel->backlight){
		panel->backlight->props.state &= ~BL_CORE_FBBLANK;
		panel->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(panel->backlight);
	}

	panel->enabled = true;

	return 0;
}

static int forlinx_panel_enable(struct drm_panel *panel)
{
	struct forlinx_panel *forlinx = to_forlinx_panel(panel);

	return forlinx->pdata->enable(forlinx);
}

static int forlinx_panel_disable(struct drm_panel *panel)
{
	struct forlinx_panel *forlinx = to_forlinx_panel(panel);

	if (!forlinx->enabled)
		return 0;

	if(forlinx->backlight){
		forlinx->backlight->props.power = FB_BLANK_POWERDOWN;
		forlinx->backlight->props.state |= BL_CORE_FBBLANK;
		backlight_update_status(forlinx->backlight);
	}

	if(forlinx->enable)
		gpiod_set_value_cansleep(forlinx->enable, 0);

	forlinx->enabled = false;

	return 0;
}

static int forlinx_panel_get_modes(struct drm_panel *panel,
			       struct drm_connector *connector)
{
	struct drm_device *drm = connector->dev;
	struct drm_display_mode *mode = NULL;
	struct forlinx_panel *forlinx = to_forlinx_panel(panel);
	struct device_node *timings_np, *np = forlinx->dsi->dev.of_node;
	int ret;

	timings_np = of_get_child_by_name(np, "display-timings");
	if(timings_np){
		mode = drm_mode_create(drm);
		if(!mode){
			of_node_put(timings_np);
			return 0;
		}
		ret = of_get_drm_display_mode(np, mode, NULL, 0);
		if(ret){
			dev_dbg(panel->dev, "failed to find dts display timings\n");
			drm_mode_destroy(drm, mode);
			mode = NULL;
		} else {
			mode->type |= DRM_MODE_TYPE_PREFERRED;
		}
		of_node_put(timings_np);
	} 

	if(!mode){
		dev_info(panel->dev, "using default display timings\n");
		mode = drm_mode_duplicate(connector->dev, &default_mode);
		if (!mode) {
		dev_err(panel->dev, "failed to add mode %ux%u@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			drm_mode_vrefresh(&default_mode));
			return -ENOMEM;
		}
		mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	}


	drm_mode_set_name(mode);
	drm_mode_probed_add(connector, mode);

	connector->display_info.width_mm = mode->width_mm;
	connector->display_info.height_mm = mode->height_mm;
	connector->display_info.bus_flags = forlinx_bus_flags;

	drm_display_info_set_bus_formats(&connector->display_info,
			forlinx_bus_formats,
			ARRAY_SIZE(forlinx_bus_formats));
	return 1;
}

static int forlinx_bl_get_brightness(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct forlinx_panel *forlinx = mipi_dsi_get_drvdata(dsi);
	u16 brightness;

	if (!forlinx->prepared)
		return 0;

//	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
//
//	ret = mipi_dsi_dcs_get_display_brightness(dsi, &brightness);
//	if (ret < 0)
//		return ret;

	bl->props.brightness = brightness;

	return brightness & 0xff;
}

static int forlinx_bl_update_status(struct backlight_device *bl)
{
	struct mipi_dsi_device *dsi = bl_get_data(bl);
	struct forlinx_panel *forlinx = mipi_dsi_get_drvdata(dsi);

	if (!forlinx->prepared)
		return 0;

//	dsi->mode_flags &= ~MIPI_DSI_MODE_LPM;
//
//	ret = mipi_dsi_dcs_set_display_brightness(dsi, bl->props.brightness);
//	if (ret < 0)
//		return ret;

	return 0;
}

static const struct backlight_ops forlinx_bl_ops = {
	.update_status = forlinx_bl_update_status,
	.get_brightness = forlinx_bl_get_brightness,
};

static const struct drm_panel_funcs forlinx_panel_funcs = {
	.prepare = forlinx_panel_prepare,
	.unprepare = forlinx_panel_unprepare,
	.enable = forlinx_panel_enable,
	.disable = forlinx_panel_disable,
	.get_modes = forlinx_panel_get_modes,
};


static const struct forlinx_platform_data forlinx_rm67191 = {
	.enable = &rm67191_enable,
};

static const struct of_device_id forlinx_panel_of_match[] = {
	{ .compatible = "forlinx,mipi-dsi", .data = &forlinx_rm67191 },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, forlinx_panel_of_match);

static int forlinx_panel_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	const struct of_device_id *of_id = of_match_device(forlinx_panel_of_match, dev);
	struct device_node *np = dev->of_node;
	struct device_node *backlight = NULL;
	struct forlinx_panel *panel;
	int ret;
	u32 video_mode;
	if (!of_id || !of_id->data)
		return -ENODEV;

	panel = devm_kzalloc(&dsi->dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, panel);

	panel->dsi = dsi;
	panel->pdata = of_id->data;

	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_EOT_PACKET;

	ret = of_property_read_u32(np, "video-mode", &video_mode);
	if (!ret) {
		switch (video_mode) {
		case 0:
			/* burst mode */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_BURST |
					   MIPI_DSI_MODE_VIDEO;
			break;
		case 1:
			/* non-burst mode with sync event */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO;
			break;
		case 2:
			/* non-burst mode with sync pulse */
			dsi->mode_flags |= MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
					   MIPI_DSI_MODE_VIDEO;
			break;
		case 3:
			/* command mode */
			dsi->mode_flags |= MIPI_DSI_CLOCK_NON_CONTINUOUS |
					   MIPI_DSI_MODE_VSYNC_FLUSH;
			break;
		default:
			dev_warn(dev, "invalid video mode %d\n", video_mode);
			break;
		}
	}

	ret = of_property_read_u32(np, "dsi-lanes", &dsi->lanes);
	if (ret) {
		dev_err(dev, "Failed to get dsi-lanes property (%d)\n", ret);
		return ret;
	}

	panel->enable = devm_gpiod_get_optional(dev, "enable",
					       GPIOD_OUT_LOW |
					       GPIOD_FLAGS_BIT_NONEXCLUSIVE);
	if (IS_ERR(panel->enable)) {
		ret = PTR_ERR(panel->enable);
		dev_err(dev, "Failed to get enable gpio (%d)\n", ret);
		return ret;
	}
	if(panel->enable)
		gpiod_set_value_cansleep(panel->enable, 0);

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if(backlight){
		panel->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);
		if(!panel->backlight)
			printk("panel without backlight\n");
	}

	if (IS_ERR(panel->backlight)) {
		ret = PTR_ERR(panel->backlight);
		dev_err(dev, "Failed to register backlight (%d)\n", ret);
		return ret;
	}

	drm_panel_init(&panel->panel, dev, &forlinx_panel_funcs,
		       DRM_MODE_CONNECTOR_DSI);
	dev_set_drvdata(dev, panel);

	drm_panel_add(&panel->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret)
		drm_panel_remove(&panel->panel);

	return ret;
}

static int forlinx_panel_remove(struct mipi_dsi_device *dsi)
{
	struct forlinx_panel *forlinx = mipi_dsi_get_drvdata(dsi);
	struct device *dev = &dsi->dev;
	int ret;

	ret = mipi_dsi_detach(dsi);
	if (ret)
		dev_err(dev, "Failed to detach from host (%d)\n", ret);

	drm_panel_remove(&forlinx->panel);

	if(forlinx->backlight)
		put_device(&forlinx->backlight->dev);
	if(forlinx->enable)
		gpiod_set_value_cansleep(forlinx->enable, 0);


	return 0;
}

static void forlinx_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct forlinx_panel *forlinx = mipi_dsi_get_drvdata(dsi);

	forlinx_panel_disable(&forlinx->panel);
	forlinx_panel_unprepare(&forlinx->panel);
}

static struct mipi_dsi_driver forlinx_panel_driver = {
	.driver = {
		.name = "panel-forlinx-mipi",
		.of_match_table = forlinx_panel_of_match,
	},
	.probe = forlinx_panel_probe,
	.remove = forlinx_panel_remove,
	.shutdown = forlinx_panel_shutdown,
};
module_mipi_dsi_driver(forlinx_panel_driver);

MODULE_AUTHOR("Robert Chiras <robert.chiras@nxp.com>");
MODULE_DESCRIPTION("DRM Driver for Forlinx MIPI DSI panel");
MODULE_LICENSE("GPL v2");
