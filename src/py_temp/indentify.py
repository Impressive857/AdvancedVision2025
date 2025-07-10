import torch
import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy import ndimage
from mobile_sam import sam_model_registry, SamAutomaticMaskGenerator

model_type = "vit_t"
sam_checkpoint = "./mobile_sam.pt"

device = "cuda" if torch.cuda.is_available() else "cpu"

mobile_sam = sam_model_registry[model_type](checkpoint=sam_checkpoint)
mobile_sam.to(device=device)
mobile_sam.eval()

mask_generator = SamAutomaticMaskGenerator(model=mobile_sam)

image = cv2.imread("table2.jpg")
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
image = cv2.resize(image, (1024, 1024))

masks = mask_generator.generate(image)
torch.cuda.empty_cache()

def keep_largest_connected_component(mask):
    """保留掩码中的最大连通区域"""
    labeled_array, num_features = ndimage.label(mask['segmentation'])
    if num_features == 0:
        return mask
    
    # 计算各组件面积
    areas = [np.sum(labeled_array == i) for i in range(1, num_features + 1)]
    largest_component = np.argmax(areas) + 1
    
    # 创建只包含最大组件的新掩码
    new_mask = np.zeros_like(mask['segmentation'])
    new_mask[labeled_array == largest_component] = 1
    
    # 更新掩码属性
    mask['segmentation'] = new_mask
    mask['area'] = np.sum(new_mask)
    return mask

def greedy_merge_overlapping(masks, overlap_threshold=0.3):
    """将小掩码合并到大掩码中"""
    # 按面积从大到小排序
    sorted_masks = sorted(masks, key=lambda x: -x['area'])
    merged_masks = []
    
    for mask in sorted_masks:
        merged = False
        for i, large_mask in enumerate(merged_masks):
            # 计算重叠比例（小掩码被覆盖的比例）
            intersection = np.logical_and(mask['segmentation'], large_mask['segmentation']).sum()
            overlap_ratio = intersection / mask['area']
            
            if overlap_ratio >= overlap_threshold:
                # 合并到现有大掩码
                merged_masks[i]['segmentation'] = np.logical_or(
                    merged_masks[i]['segmentation'], mask['segmentation']
                )
                merged_masks[i]['area'] = np.sum(merged_masks[i]['segmentation'])
                merged = True
                break
                
        if not merged:
            merged_masks.append(mask.copy())
    
    return merged_masks

masks = [m for m in masks if m['area'] < 200000]
# 应用到所有掩码
masks = [keep_largest_connected_component(m) for m in masks]

masks = greedy_merge_overlapping(masks)

masks = [m for m in masks if m['area'] > 6000]

# 创建子图网格
n = len(masks) + 1  # +1 是为了原图
rows = int(np.ceil(n / 3))  # 每行显示3个子图
fig, axes = plt.subplots(rows, 3, figsize=(15, rows * 5))
axes = axes.flatten()

# 显示原图
axes[0].imshow(image)
axes[0].set_title("Original Image")
axes[0].axis("off")

# 显示每个掩码
for i, mask in enumerate(masks):
    ax = axes[i + 1]
    # 创建彩色掩码
    color_mask = np.zeros_like(image)
    color = [0, 0, 0]
    color_mask[mask["segmentation"]] = color

    # 叠加掩码到原图
    overlay = image.copy()
    alpha = 0.25
    overlay[mask["segmentation"]] = (
        alpha * overlay[mask["segmentation"]]
        + (1 - alpha) * color_mask[mask["segmentation"]]
    )

    ax.imshow(overlay)
    ax.set_title(f'Mask {i+1} (Area: {mask["area"]:.1f})')
    ax.axis("off")

# 隐藏空白子图
for i in range(n, len(axes)):
    axes[i].axis("off")

plt.tight_layout()
plt.show()