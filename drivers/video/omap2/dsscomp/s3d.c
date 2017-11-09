/*
 * linux/drivers/video/omap2/dsscomp/s3d.c
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


#include <video/omapdss.h>
#include <video/dsscomp.h>
#include <plat/dsscomp.h>

#include "dsscomp.h"
#include "s3d.h"

static int needed;

/* We scale all 2D layers to half the size. Each view of 3D has
 * half the height of the display in case of row interleaving.
 */
int s3d_set_crop_coords(struct dss2_ovl_info *oi,
		struct omap_dss_device *dssdev,
		struct device *dev,
		int pass)
{
	struct s3d_disp_info *s3d_info = &dssdev->panel.s3d_info;

	bool need_left_view = true;
	enum where {
		left    = 1,
		right   = 0xFFFFFFFe,
		top     = 2,
		bottom  = 0xFFFFFFFd,
	};
	enum where crop_here;

	if (!oi || !s3d_info)
		return -EINVAL;

	if (pass == 1) {
		if (s3d_info->order == S3D_DISP_ORDER_L)
			need_left_view = true;
		else
			need_left_view = false;
	} else if (pass == 2) {
		if (s3d_info->order == S3D_DISP_ORDER_L)
			need_left_view = false;
		else
			need_left_view = true;
	}

	if (oi->cfg.s3d_input_layout_type == S3D_SIDE_BY_SIDE) {
		if (pass == 1)
			oi->cfg.crop.w = oi->cfg.crop.w/2;
		if (oi->cfg.s3d_input_layout_order == LEFT_VIEW_FIRST)
			crop_here = left;
		else
			crop_here = right;
	} else if (oi->cfg.s3d_input_layout_type == S3D_TOP_BOTTOM) {
		if (pass == 1)
			oi->cfg.crop.h = oi->cfg.crop.h/2;
		if (oi->cfg.s3d_input_layout_order == LEFT_VIEW_FIRST)
			crop_here = top;
		else
			crop_here = bottom;
	} else {
		dev_err(dev, "Unsupported input layout for WB S3D\n");
		return -EINVAL;
	}

	if (!need_left_view)
		crop_here = ~crop_here;

	switch (crop_here) {
	case (left):
	case (top):
		{
			/* First pass on left/top, no modification to crop */
			break;
		}
	case (right):
		{
			oi->cfg.crop.x = oi->cfg.crop.x + oi->cfg.crop.w;
			break;
		}
	case (bottom):
		{
			oi->cfg.crop.y = oi->cfg.crop.y + oi->cfg.crop.h;
			break;
		}
	default:
		{
			dev_err(dev, "crop_here: error\n");
			return -EINVAL;
		}
	}
	return crop_here;
}

int s3d_crop(struct dss2_ovl_info *oi, int crop_here, struct device *dev)
{
	enum where {
		left    = 1,
		right   = 0xFFFFFFFe,
		top     = 2,
		bottom  = 0xFFFFFFFd,
	};

	if (!oi)
		return -EINVAL;

	/* This is pass 2. So reset the first crop changes. */
	switch (crop_here) {
	case(left):
		{
			oi->cfg.crop.x = (oi->cfg.crop.x > oi->cfg.crop.w) ?
				(oi->cfg.crop.x - oi->cfg.crop.w) : 0;
			break;
		}
	case(top):
		{
			oi->cfg.crop.y = (oi->cfg.crop.y > oi->cfg.crop.h) ?
				(oi->cfg.crop.y - oi->cfg.crop.h) : 0;
			break;
		}
	case(right):
		{
			oi->cfg.crop.x = oi->cfg.crop.x + oi->cfg.crop.w;
			break;
		}
	case(bottom):
		{
			oi->cfg.crop.y = oi->cfg.crop.y + oi->cfg.crop.h;
			break;
		}
	default:
		{
			dev_err(dev, "crop_here: error\n");
		}
	}

	return 0;
}

/* As part of WB, we have connected VID1, VID2, VID3 pipelines to source_mgr.
 * GFX pipeline is connected to wb_dest_mgr.
 * We turn off all these overlays forcefully, in case of blanking composition.
 */
int apply_turnoff_ovls_on_wb_src_mgr(
	struct s3d_composition_data *s3d_comp_data,
	struct omap_overlay_manager *wb_source_mgr,
	struct omap_overlay_manager *wb_dest_mgr)
{
	int ovl_idx = 0;
	int number_of_overlays = omap_dss_get_num_overlays();
	struct omap_overlay_info ovl_info;
	struct omap_overlay *ovl;
	struct device *dev = s3d_comp_data->dev;

	needed = 0;

	for (ovl_idx = 0; ovl_idx < number_of_overlays; ovl_idx++) {
		ovl = omap_dss_get_overlay(ovl_idx);
		if (!ovl) {
			dev_err(dev, "%s:ovl == NULL\n", __func__);
			return -ENXIO;
		}
		ovl->get_overlay_info(ovl, &ovl_info);
		ovl->info.enabled = false;
	}

	if (!wb_source_mgr) {
		dev_err(dev, "%s:wb_source_mgr NULL\n", __func__);
		return -ENXIO;
	}
	if (wb_source_mgr->apply(wb_source_mgr)) {
		dev_err(dev, "%s:failed while wb_src_mgr[%d] apply",
				__func__, wb_source_mgr->id);
		return -ENXIO;
	}
	return 0;
}

/* As part of WB mem2mem interleaving, we connected VIDx pipelines to
 * wb_source_mgr. Connect them back to wb_dest when we transition from 3D to 2D
 * (no WB). wb_dest_mgr is the manager that is actually connected to the
 * display.
 */
int connect_back_forced_ovls(struct dsscomp_data *comp,
		struct omap_overlay_manager *wb_source_mgr,
		struct omap_overlay_manager *wb_dest_mgr,
		struct device *dev)
{
	int who_is_on[4];
	int ovl_idx = 0;
	int number_of_overlays = omap_dss_get_num_overlays();
	struct omap_overlay_info ovl_info;
	struct omap_overlay *ovl;
	if (!wb_source_mgr || !wb_dest_mgr)
		return -EINVAL;

	memset(who_is_on, 0, 4 * sizeof(int));

	/* Identify the overlays that are ON for current composition */
	for (ovl_idx = 0; ovl_idx < number_of_overlays; ovl_idx++) {
		struct dss2_ovl_info *oi = comp->ovls + ovl_idx;
		if (oi->cfg.enabled)
			who_is_on[oi->cfg.ix] = 1;
	}

	for (ovl_idx = 0; ovl_idx < number_of_overlays; ovl_idx++) {
		ovl = omap_dss_get_overlay(ovl_idx);
		if (!ovl) {
			dev_err(dev, "%s:ovl == NULL\n", __func__);
			return -EINVAL;
		}
		ovl->get_overlay_info(ovl, &ovl_info);
		ovl->manager = NULL;
		/* Turn off overlays (required to change manager) */
		if (ovl->info.enabled)
			ovl->info.enabled = false;

		/* Change manager */
		if (ovl->set_manager(ovl, wb_dest_mgr))
			return -EINVAL;

		/* If current composition needs this ovly on, turn it on. */
		if (who_is_on[ovl_idx])
			ovl->info.enabled = true;
	}
	if (wb_source_mgr->apply(wb_source_mgr)) {
			dev_err(dev,
				"WB: state transition wb_dest_mgr[%d] error\n",
				wb_source_mgr->id);
			return -EINVAL;
	}

	return 0;
}
/* In case of WB MEM2MEM ROW_INTERLEAVING mode, use GFX pipeline to display the
 * output of WB.
 */
int configure_display_wb_output(unsigned long buf_phys_addr,
		unsigned int width, unsigned int height,
		struct omap_overlay_manager *wb_dest_mgr,
		struct device *dev)
{
	struct omap_overlay *ovl_vid;
	struct omap_overlay_info ovl_vid_info;

	ovl_vid = omap_dss_get_overlay(OMAP_DSS_GFX);
	if (!ovl_vid) {
		dev_err(dev, "%s: ovl_vid == NULL\n", __func__);
		return -EINVAL;
	}

	ovl_vid->get_overlay_info(ovl_vid, &ovl_vid_info);
	ovl_vid_info.paddr = buf_phys_addr;
	ovl_vid_info.enabled       = true;
	ovl_vid_info.width         = width;
	ovl_vid_info.height        = height;
	ovl_vid_info.out_width     = ovl_vid_info.width;
	ovl_vid_info.out_height    = ovl_vid_info.height;
	ovl_vid_info.pos_x         = 0;
	ovl_vid_info.pos_y         = 0;
	ovl_vid_info.color_mode    = OMAP_DSS_COLOR_RGBX32;
	ovl_vid_info.min_x_decim   = 1;
	ovl_vid_info.min_y_decim   = 1;
	ovl_vid_info.max_x_decim   = 1;
	ovl_vid_info.max_y_decim   = 1;
	ovl_vid_info.rotation      = 0;
	ovl_vid_info.mirror        = 0;
	ovl_vid_info.global_alpha  = 255;
	ovl_vid_info.screen_width  = width; /* Assuming ROW_INTERLEAVING */
	ovl_vid_info.zorder        = OMAP_DSS_OVL_ZORDER_0;
	ovl_vid_info.rotation_type = OMAP_DSS_ROT_DMA;

	if (ovl_vid->set_overlay_info(ovl_vid, &ovl_vid_info)) {
		dev_err(dev, "%s: ERROR at %d %s\n",
				__func__, __LINE__, __FILE__);
		return -EINVAL;
	}

	if (!wb_dest_mgr) {
		dev_err(dev, "%s: wb_dest_mgr is NULL\n", __func__);
		return -EINVAL;
	}
	if (wb_dest_mgr->apply(wb_dest_mgr)) {
		dev_err(dev, "%s: WB: disp: wb_dest_mgr[%d]->apply failed\n",
			__func__, wb_dest_mgr->id);
		return -EINVAL;
	}
	return 0;
}



/* Configure write-back with the given parameters. */
int configure_wb_params(
		struct s3d_composition_data *s3d_comp_data,
		enum dsscomp_composition_mode comp_mode,
		unsigned int wb_source_manager_id,
		struct omap_writeback *wb, struct dss2_wb_params *wb_params,
		unsigned long *wb_output_buf_pa, int mono_to_stereo)
{
	int r;
	unsigned long *wb_output_buf_size = NULL;
	unsigned long *paddr1 = NULL;
	unsigned long *paddr2 = NULL;
	unsigned long *vaddr1 = NULL;
	unsigned long *vaddr2 = NULL;
	unsigned int use_pong = 0;
	unsigned long phys_addr1, phys_addr2, virt_addr1, virt_addr2;
	struct device *dev = s3d_comp_data->dev;


	wb_output_buf_size = &(s3d_comp_data->wb_output_buf_size);
	paddr1 = &(s3d_comp_data->phys_addr1);
	paddr2 = &(s3d_comp_data->phys_addr2);
	vaddr1 = &(s3d_comp_data->virt_addr1);
	vaddr2 = &(s3d_comp_data->virt_addr2);
	use_pong = s3d_comp_data->use_pong;

	if ((comp_mode == DSSCOMP_WB_M2M_ROW_INTERLEAVED) ||
			(comp_mode == DSSCOMP_WB_M2M_COL_INTERLEAVED))
		wb_params->wb_mode = OMAP_WB_MEM2MEM_MODE;
	else
		wb_params->wb_mode = OMAP_WB_CAPTURE_MODE;

	wb_params->enable = true;
	wb_params->source = wb_source_manager_id;
	wb_params->line_skip = 0;

	if (comp_mode == DSSCOMP_WB_M2M_ROW_INTERLEAVED)
		wb_params->line_skip = 1;

	wb_params->width = wb->width;
	wb_params->height = wb->height;

	if (wb_params->line_skip)
		wb_params->height = wb->height/2;

	wb_params->color_mode = OMAP_DSS_COLOR_RGBX32;
	/* Since we are using RGB */
	wb_params->p_uv_addr = 0;

	/* Allocate memory the first time an S3D surface becomes visible.*/
	if (*wb_output_buf_size == 0) {
		/* In ROW_INTERLEAVED composition mode, input to WB is always
		 * the output of a manager (in memory to memory mode). The
		 * output of a manager is always in RGB format. Hence the bytes
		 * per pixel is hard-coded here to 4.
		 */
		*wb_output_buf_size = wb->width * wb->height * 4;

		virt_addr1 = (unsigned long)alloc_pages_exact(
				*wb_output_buf_size, GFP_KERNEL | GFP_DMA);
		virt_addr2 = (unsigned long)alloc_pages_exact(
				*wb_output_buf_size, GFP_KERNEL | GFP_DMA);
		if (!virt_addr1 || !virt_addr2) {
			dev_err(dev, "%s: WB o/p buf allocation failed.",
					__func__);
			*wb_output_buf_size = 0;
			wb_params->enable = false;
			return -ENOMEM;
		}

		/* Things we do when allocation goes through*/
		phys_addr1 = virt_to_phys((void *)virt_addr1);
		phys_addr2 = virt_to_phys((void *)virt_addr2);
		if (!phys_addr1 || !phys_addr2) {
			dev_err(dev, "%s: phys_addr is NULL\n", __func__);
			return -EINVAL;
		}

		*paddr1 = phys_addr1;
		*paddr2 = phys_addr1;
		*vaddr1 = virt_addr1;
		*vaddr2 = virt_addr2;

		if (!use_pong)
			*wb_output_buf_pa = phys_addr1;
		else
			*wb_output_buf_pa = phys_addr2;
	} else {
		if (*paddr1 == 0 || *paddr2 == 0) {
			dev_err(dev, "%s: phys_addr == NULL\n", __func__);
			return -EINVAL;
		} else {
			if (!use_pong)
				*wb_output_buf_pa = *paddr1;
			else
				*wb_output_buf_pa = *paddr2;
		}
	}

	if (mono_to_stereo) {
		r = omap_dispc_register_isr(dss_wb_framedone_cb, NULL,
				DISPC_IRQ_FRAMEDONE_WB);
		if (r)
			dev_err(dev, "%s:failed to register WB framedone ISR\n",
					__func__);
	}

	wb_params->output_buf_pa = *wb_output_buf_pa;
	return 0;
}

/* Find the manager that is not connected to any display. That
 * would be the source of WB.
 */
int get_free_lcd_mgr(struct device *dev,
		unsigned int num_displays, unsigned int num_mgrs,
		struct omap_dss_device **displays,
		struct omap_writeback *wb,
		struct omap_overlay_manager **wb_source_mgr_arg,
		struct omap_overlay_manager **wb_dest_mgr_arg,
		int *wb_source_mgr_id, int *wb_dest_mgr_id)
{
	int display_idx;
	/* Flag to mark which of the 3 managers are currently connected
	 * to an available display.
	 */
	unsigned int mgrs_in_use = 0;
	struct omap_overlay_manager *wb_source_mgr = NULL;
	struct omap_overlay_manager *wb_dest_mgr = NULL;

	if (!wb) {
		dev_err(dev, "%s: wb NULL\n", __func__);
		return -ENXIO;
	}


	for (display_idx = 0; display_idx < num_displays;
			display_idx++) {
		const struct omap_dss_device *dss_device;
		struct omap_overlay_manager *wb_mgr = NULL;

		dss_device = displays[display_idx];
		if (!dss_device)
			continue;
		wb_mgr = dss_device->manager;
		if (wb_mgr->id >= num_mgrs) {
			dev_err(dev, "%s:disp[%d] id > num_mgrs\n",
					__func__, display_idx);
			dev_err(dev, "WB_M2M_COMP:disp[%d]"
					" getting manager error\n",
					display_idx);
			return -ENXIO;
		} else {
			mgrs_in_use |= (1 << dss_device->manager->id);
			if (wb_mgr
					&& wb_mgr->id != OMAP_DSS_OVL_MGR_TV) {
				*wb_dest_mgr_id = wb_mgr->id;
				wb->width =
					dss_device->panel.timings.x_res;
				wb->height =
					dss_device->panel.timings.y_res;
			}

		}
	}

	if (!(mgrs_in_use & 0x1))
		*wb_source_mgr_id = 0;
	else if (!(mgrs_in_use & 0x4))
		*wb_source_mgr_id = 2;
	else if (!(mgrs_in_use & 0x2) || (*wb_source_mgr_id >= 3)) {
		dev_err(dev, "WB_M2M_COMP: finding free manager"
				" failed: mgrs_in_use=0x%x\n",
				mgrs_in_use);
		return -ENXIO;
	}

	wb_source_mgr = omap_dss_get_overlay_manager(
			*wb_source_mgr_id);
	wb_dest_mgr = omap_dss_get_overlay_manager(
			*wb_dest_mgr_id);
	if (!wb_source_mgr || !wb_dest_mgr) {
		dev_err(dev, "wb_src_mgr=%d dest=%d NULL\n",
				*wb_source_mgr_id,
				*wb_dest_mgr_id);
		return -ENXIO;
	}
	if (!wb_source_mgr->device)
		wb_source_mgr->device = wb_dest_mgr->device ?
			wb_dest_mgr->device : NULL;
	else if (!wb_dest_mgr->device)
		wb_dest_mgr->device = wb_source_mgr->device ?
			wb_source_mgr->device : NULL;

	if (!wb_source_mgr->device ||
			!wb_dest_mgr->device) {
		dev_err(dev, "wb_src%d/dest%d_mgr device NULL\n",
				*wb_source_mgr_id,
				*wb_dest_mgr_id);
		return -ENXIO;
	}

	if (!wb_source_mgr || !wb_dest_mgr) {
		dev_err(dev, "%s: wb_source/dest_mgr is NULL\n",
				__func__);
		return -ENXIO;
	}

	*wb_source_mgr_arg = wb_source_mgr;
	*wb_dest_mgr_arg = wb_dest_mgr;

	return 0;
}

int s3d_reset_to_mono(struct s3d_composition_data **s3d_comp_data)
{
	int r = 0;
	struct device *dev = (*s3d_comp_data)->dev;

	/* Unregister callback registered for WB framedone. */
	r = omap_dispc_unregister_isr(dss_wb_framedone_cb, NULL,
			DISPC_IRQ_FRAMEDONE_WB);
	if (r)
		dev_err(dev, "%s: ISR unregister returned %d\n",
				__func__, r);

	needed = 0;


#if 0
	/* The WB output buffer is almost 2000 pages in size (for 1280*800 size
	 * display). This buffer needs to be physically contiguous. Allocating
	 * and freeing it upon each S3D content start can cause system to run
	 * into fragmentation very soon. Here we allocate the buffer once the
	 * S3D has been used, and then it remains in the system. The other
	 * option would be a static allocation using memblock (like a carve
	 * out). That is going to stay in the system, even if S3D is used or
	 * not.
	 * In short: the #if 0, is to remind that there are buffers to be freed,
	 * but are kept not-freed to avoid fragmentation issues.
	 */
	if ((*s3d_comp_data)->wb_output_buf_size &&
			(*s3d_comp_data)->virt_addr1) {
		free_pages_exact((void *)(*s3d_comp_data)->virt_addr1,
				(*s3d_comp_data)->wb_output_buf_size);

		(*s3d_comp_data)->phys_addr1 = 0;
		(*s3d_comp_data)->virt_addr1 = 0;

		dev_info(dev, "%s: ping freed\n", __func__);
	}

	if ((*s3d_comp_data)->wb_output_buf_size &&
			(*s3d_comp_data)->virt_addr2) {
		free_pages_exact((void *)(*s3d_comp_data)->virt_addr2,
				(*s3d_comp_data)->wb_output_buf_size);
		(*s3d_comp_data)->phys_addr2 = 0;
		(*s3d_comp_data)->virt_addr2 = 0;
		dev_info(dev, "%s: p0ng freed\n", __func__);
	}

	/* Setting size parameter to zero. This will trigger an allocation next
	 * time 3D composition starts.
	 */
	(*s3d_comp_data)->wb_output_buf_size = 0;

	kfree(*s3d_comp_data);
#endif

	return r;
}

int s3d_wb_init(struct device *dev,
		struct s3d_composition_data **s3d_comp_data,
		struct omap_writeback *wb,
		int num_displays, int num_mgrs,
		struct omap_dss_device **displays,
		enum dsscomp_composition_mode prev_comp_mode,
		enum dsscomp_composition_mode composition_mode,
		unsigned long *wb_output_buf_pa,
		struct omap_overlay_manager *mgr)
{
	int r = 0;

	struct dss2_wb_params wb_param;
	struct dss2_wb_params *wb_params = &wb_param;

	int mono_to_stereo = 0;

	needed = 1;

	if (!*s3d_comp_data) {

		*s3d_comp_data = kzalloc(
				sizeof(struct s3d_composition_data),
				GFP_KERNEL);
		if (!*s3d_comp_data) {
			dev_err(dev, "s3d_comp_data allocation fail\n");
			return -ENXIO;
		}

		(*s3d_comp_data)->dev = dev;
	}

	r = get_free_lcd_mgr((*s3d_comp_data)->dev,
			num_displays, num_mgrs, displays, wb,
			&((*s3d_comp_data)->wb_source_mgr),
			&((*s3d_comp_data)->wb_dest_mgr),
			&((*s3d_comp_data)->wb_source_mgr_id),
			&((*s3d_comp_data)->wb_dest_mgr_id));

	if (r || !(*s3d_comp_data)->wb_source_mgr
			|| !(*s3d_comp_data)->wb_dest_mgr) {
		dev_err(dev, "get_free_lcd_mgr() returned %d\n", r);
		return -EINVAL;
	}

	if ((prev_comp_mode == DSSCOMP_NORMAL_COMPOSITION) &&
			(composition_mode == DSSCOMP_WB_M2M_ROW_INTERLEAVED))
		mono_to_stereo = 1;

	/* Configure WB parameters */
	configure_wb_params(*s3d_comp_data, composition_mode,
			(*s3d_comp_data)->wb_source_mgr_id, wb, wb_params,
			wb_output_buf_pa, mono_to_stereo);
	if (!wb_output_buf_pa) {
		dev_err(dev, "wb_output_buf_pa is NULL\n");
		r = -ENOMEM;
	}

	/* Toggle ping-pong */
	(*s3d_comp_data)->use_pong = !(*s3d_comp_data)->use_pong;

	/* Fill the parameters into omap_write_back_info structure */
	r = set_dss_wb_params(wb_params, mgr);
	if (r) {
		dev_err(dev, "%s: set_dss_wb_params returned %d\n",
				__func__, r);
		return r;
	}

	return 0;
}
int s3d_wb_pass2(struct s3d_composition_data **s3d_comp_data,
		struct dsscomp_data *comp,
		struct omap_writeback *wb,
		struct omap_writeback *wb_ovl,
		unsigned long wb_output_buf_pa,
		struct device *dev)
{
	int oix;
	int r = 0;

	for (oix = 0; oix < comp->frm.num_ovls; oix++) {
		struct dss2_ovl_info *oi = comp->ovls + oix;
		int *crop_coords = &((*s3d_comp_data)->crop_coords);

		if (oi->cfg.s3d_content) {
			/* Change crop parameter */
			r = s3d_crop(oi, ~(*crop_coords), dev);
			if (r) {
				dev_err(dev, "%s: s3d_crop() returned %d\n",
						__func__, r);
				return -EINVAL;
			}

			r = set_dss_ovl_info(oi);
			if (r) {
				dev_err(dev, "%s: set_ovl_info() returned %d",
						__func__, r);
				return -EINVAL;
			}
		}
	}

	/* Apply the new crop parameter through a manager-apply. */
	if (!(*s3d_comp_data)->wb_source_mgr) {
		dev_err(dev, "%s: wb_source manager NULL\n",\
				__func__);
		return -EINVAL;
	}

	(*s3d_comp_data)->wb_source_mgr->apply((*s3d_comp_data)->wb_source_mgr);

	/* TODO: JBP: 4 to bpp. */
	/* Writeback output should point to the next row for pass 2. */
	wb->info.paddr = wb_output_buf_pa + 4 * wb->width;

	/* Start the WB again, on the new (pass2) parameters. */
	omap_dss_wb_apply((*s3d_comp_data)->wb_source_mgr, wb_ovl);

	return 0;
}



/* This is the crux of DSSCOMP_WBM2M_ROW_INTERLEAVED
 * composition mode. We connect WB to the source
 * manager, and all overlays except GFX to the source
 * manager. Output buffer of WB is shown through GFX
 * ovly which is connected to the wb_destination
 * manager (which inturn is connected to the display).
 */
int s3d_wb_pass1(struct s3d_composition_data **s3d_comp_data,
		struct omap_writeback *wb_ovl,
		struct device *dev)

{

	int r = -1;
	int ovl_idx = 0;
	bool we_turned_off;
	int number_of_overlays = omap_dss_get_num_overlays();
	struct omap_overlay_info ovl_info;
	struct omap_overlay *ovl;

	for (ovl_idx = 0; ovl_idx < number_of_overlays; ovl_idx++) {
		/* Get ovly info of all ovls */
		ovl = omap_dss_get_overlay(ovl_idx);
		if (!ovl) {
			dev_err(dev, ":%s:ovl == NULL\n", __func__);
			return -EINVAL;
		}

		ovl->get_overlay_info(ovl, &ovl_info);

		/* Set enabled = 0 for every pipeline */

		we_turned_off = false;

		if (ovl_info.enabled) {
			ovl->info.enabled = false;
			we_turned_off = true;
		}

		/* Unset current manager off all pipelines */
		ovl->manager = NULL;

		/* Connect all pipelines to source mgr connected
		 * to WB input.
		 */
		if (ovl_idx != 0) {
			r = ovl->set_manager(ovl,
					(*s3d_comp_data)->wb_source_mgr);
			if (r) {
				dev_err(dev, "%s: set_manager(%d) ret %d\n",
						__func__, ovl_idx, r);
				return r;
			}
		} else{
			/* Connect GFX to mgr connected to display. */
			r = ovl->set_manager(ovl,
					(*s3d_comp_data)->wb_dest_mgr);
			if (r) {
				dev_err(dev, "%s: set_manager(%d) ret %d\n",
						__func__, ovl_idx, r);
				return r;
			}
		}

		if (we_turned_off) {
			/* GFX will be turned on later */
			if (ovl_idx != 0)
				ovl->info.enabled = true;
		}
	}

	/* configure WB */
	r = omap_dss_wb_apply((*s3d_comp_data)->wb_source_mgr, wb_ovl);
	if (r) {
		dev_err(dev, "%s: failed during omap_dss_wb_apply %d",
				__func__, r);
		return r;
	}

	needed = 1;

	r = (*s3d_comp_data)->wb_source_mgr->apply(
			(*s3d_comp_data)->wb_source_mgr);
	if (r) {
		dev_err(dev, "%s:failed on src mgr[%d] apply %d",
				__func__, r,
				(*s3d_comp_data)->wb_source_mgr->id);
	}

	return 0;
}
void dss_wb_framedone_cb(void *data, u32 mask)
{
	if (needed == 1) {
		needed = 0;
		dispc_enable_plane(OMAP_DSS_WB, true);
	}
}
