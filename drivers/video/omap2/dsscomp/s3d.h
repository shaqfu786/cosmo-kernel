/*
 * linux/drivers/video/omap2/dsscomp/s3d.h
 *
 * DSS Composition S3D support.
 *
 * Copyright (C) 2012 Texas Instruments, Inc
 * Author: Jagadeesh Pakaravoor <j-pakaravoor@ti.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */


struct dsscomp_dev;

struct s3d_composition_data {	/*For print purposes */
	struct device *dev;
	/* mark a necessary WB framedone interrupt */
	int needed;
	/* ping buffer */
	unsigned long phys_addr1;
	unsigned long virt_addr1;
	/* pong buffer */
	unsigned long phys_addr2;
	unsigned long virt_addr2;
	/* Size of WB output buffer. We have 2 of these. It normally matches
	 * the size of the local display.
	 */
	unsigned long wb_output_buf_size;
	/* Crop location - left/right/top/bottom.
	 * TODO: We need to make this an array, if we need to support more than
	 * one S3D layer and they have a different layouts.
	 */
	int crop_coords;
	unsigned int use_pong;
	/* WB's source, not connected to any display */
	struct omap_overlay_manager *wb_source_mgr;
	int wb_source_mgr_id;
	/* WB's destination, connected to local display */
	struct omap_overlay_manager *wb_dest_mgr;
	int wb_dest_mgr_id;

};

int s3d_set_crop_coords(struct dss2_ovl_info *oi,
	struct omap_dss_device *dssdev,
	struct device *dev, int pass);


int apply_turnoff_ovls_on_wb_src_mgr(
	struct s3d_composition_data *s3d_comp_data,
	struct omap_overlay_manager *wb_source_mgr,
	struct omap_overlay_manager *wb_dest_mgr);

int connect_back_forced_ovls(struct dsscomp_data *comp,
	struct omap_overlay_manager *wb_source_mgr,
	struct omap_overlay_manager *wb_dest_mgr,
	struct device *dev);

int configure_display_wb_output(unsigned long buf_phys_addr,
	unsigned int width, unsigned int height,
	struct omap_overlay_manager *wb_dest_mgr,
	struct device *dev);

int s3d_reset_to_mono(struct s3d_composition_data **s3d_comp_data);

int s3d_wb_init(struct device *dev,
		struct s3d_composition_data **s3d_comp_data,
		struct omap_writeback *wb,
		int num_displays, int num_mgrs,
		struct omap_dss_device **displays,
		enum dsscomp_composition_mode prev_comp_mode,
		enum dsscomp_composition_mode composition_mode,
		unsigned long *wb_output_buf_pa,
		struct omap_overlay_manager *mgr);


int s3d_wb_pass1(struct s3d_composition_data **data,
		struct omap_writeback *wb_ovl,
		struct device *dev);

int s3d_wb_pass2(struct s3d_composition_data **s3d_comp_data,
		struct dsscomp_data *comp,
		struct omap_writeback *wb,
		struct omap_writeback *wb_ovl,
		unsigned long wb_output_buf_pa,
		struct device *dev);

