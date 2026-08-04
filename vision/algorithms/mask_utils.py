# -*- coding: utf-8 -*-
"""
掩码工具 (Mask Utilities)

连通域清洗: 保留触底最大连通域，返回主赛道mask和噪点mask。
"""

import cv2
import numpy as np


def clean_mask_by_cc(mask: np.ndarray, min_bottom_y: int):
    """
    连通域清洗 (底层锚定法):
    1. cv2.connectedComponentsWithStats 获取连通域
    2. 触底校验: bounding box 底部 y+h >= min_bottom_y
    3. 面积最大: 在满足触底条件的连通域中保留面积最大的
    4. 返回 (clean_mask, noise_mask)

    :return: (clean_mask, noise_mask) — 均为 uint8 np.ndarray
    """
    if mask is None or mask.size == 0:
        return mask, None

    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)

    if num_labels <= 1:
        return np.zeros_like(mask), None

    candidate_idx = -1
    candidate_area = -1

    for i in range(1, num_labels):
        y = stats[i, cv2.CC_STAT_TOP]
        hh = stats[i, cv2.CC_STAT_HEIGHT]
        bottom = y + hh
        area = stats[i, cv2.CC_STAT_AREA]

        if bottom >= min_bottom_y and area > candidate_area:
            candidate_area = area
            candidate_idx = i

    clean_mask = np.zeros_like(mask)
    if candidate_idx > 0:
        clean_mask[labels == candidate_idx] = 255

    noise_mask = np.zeros_like(mask)
    noise_mask[(mask == 255) & (clean_mask == 0)] = 255

    return clean_mask, noise_mask
