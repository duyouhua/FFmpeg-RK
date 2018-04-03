/*
 * Copyright 2017 Rockchip Electronics Co., Ltd
 *     Author: James Lin<james.lin@rock-chips.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Library General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Library General Public License for more details.
 *
 * You should have received a copy of the GNU Library General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 51 Franklin St, Fifth Floor,
 * Boston, MA 02110-1301, USA.
 *
 */

#include "rkvdec.h"

typedef struct RKVDECPicParamsH264 {
    RK_U32  frame_width;
    RK_U32  frame_height;
    RK_U16  frame_width_in_mbs_minus1;
    RK_U16  frame_height_in_mbs_minus1;
    RK_U8   num_ref_frames;

    union {
        struct {
            RK_U16  field_pic_flag                  : 1;
            RK_U16  mbaff_frame_flag                : 1;
            RK_U16  residual_colour_transform_flag  : 1;
            RK_U16  sp_for_switch_flag              : 1;
            RK_U16  chroma_format_idc               : 2;
            RK_U16  ref_pic_flag                    : 1;
            RK_U16  constrained_intra_pred_flag     : 1;

            RK_U16  weighted_pred_flag              : 1;
            RK_U16  weighted_bipred_idc             : 2;
            RK_U16  mbs_consecutive_flag            : 1;
            RK_U16  frame_mbs_only_flag             : 1;
            RK_U16  transform_8x8_mode_flag         : 1;
            RK_U16  min_luma_bipred_size8x8_flag    : 1;
            RK_U16  IntraPicFlag                    : 1;
        };
        RK_U16  bit_fields;
    };

    RK_U8   bit_depth_luma_minus8;
    RK_U8   bit_depth_chroma_minus8;

    RKVDECPicture   curr_pic;
    RK_U32          curr_mv;
    
    RKVDECPicture   DPB[16];
    RKVDECPicture   ref_frame_list[2][32];
    RK_U32          ref_colmv_list[16];

    RK_S16  pic_init_qs_minus26;
    RK_S16  chroma_qp_index_offset;
    RK_S16  second_chroma_qp_index_offset;
    RK_S16  pic_init_qp_minus26;
    
    RK_U8   num_ref_idx_l0_active_minus1;
    RK_U8   num_ref_idx_l1_active_minus1;
    RK_U16  frame_num;

    RK_U8   log2_max_frame_num_minus4;
    RK_U8   pic_order_cnt_type;
    RK_U8   log2_max_pic_order_cnt_lsb_minus4;
    RK_U8   delta_pic_order_always_zero_flag;

    RK_U8   direct_8x8_inference_flag;
    RK_U8   entropy_coding_mode_flag;
    RK_U8   pic_order_present_flag;
    RK_U8   num_slice_groups_minus1;

    RK_U8   slice_group_map_type;
    RK_U8   deblocking_filter_control_present_flag;
    RK_U8   redundant_pic_cnt_present_flag;

    RK_U8   scaleing_list_enable_flag;
    RK_U8   scaling_lists4x4[6][16];
    RK_U8   scaling_lists8x8[6][64];

    /* Following are H.264 MVC Specific parameters */
    RK_U8   num_views_minus1;
    RK_U16  view_id[16];
    RK_U8   num_anchor_refs_l0[16];
    RK_U16  anchor_ref_l0[16][16];
    RK_U8   num_anchor_refs_l1[16];
    RK_U16  anchor_ref_l1[16][16];
    RK_U8   num_non_anchor_refs_l0[16];
    RK_U16  non_anchor_ref_l0[16][16];
    RK_U8   num_non_anchor_refs_l1[16];
    RK_U16  non_anchor_ref_l1[16][16];

    RK_U16  curr_view_id;
    RK_U8   anchor_pic_flag;
    RK_U8   inter_view_flag;
    RK_U16  view_id_list[16];

    RK_U16  curr_layer_id;
    RK_U8   ref_pic_layerid_list[16];
}RKVDECPicParamsH264;

