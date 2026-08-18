"""3.4 可选真实推理：官方Grounding DINO + SAM。

该脚本不属于默认依赖。先按README安装官方仓库并下载权重，再运行本入口。
"""

import argparse
from pathlib import Path

import cv2
import numpy as np


def main():
    """用Grounding DINO检测框提示SAM，并把合并Mask保存为PNG。"""
    parser = argparse.ArgumentParser(description="Grounding DINO框提示SAM分割")
    parser.add_argument("--image", required=True)
    parser.add_argument("--prompt", required=True)
    parser.add_argument("--grounding-config", required=True)
    parser.add_argument("--grounding-checkpoint", required=True)
    parser.add_argument("--sam-checkpoint", required=True)
    parser.add_argument("--sam-type", default="vit_b", choices=["vit_b", "vit_l", "vit_h"])
    parser.add_argument("--box-threshold", type=float, default=0.35)
    parser.add_argument("--text-threshold", type=float, default=0.25)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--output", default="grounded_sam_mask.png")
    args = parser.parse_args()

    try:
        from groundingdino.util.inference import load_image, load_model, predict
        from segment_anything import SamPredictor, sam_model_registry
    except ImportError as error:
        raise SystemExit(
            "缺少可选依赖。请阅读docs/chapter03/OPTIONAL_MODELS.md。"
        ) from error

    grounding_model = load_model(args.grounding_config, args.grounding_checkpoint)
    image_rgb, grounding_input = load_image(args.image)
    boxes, scores, phrases = predict(
        model=grounding_model,
        image=grounding_input,
        caption=args.prompt,
        box_threshold=args.box_threshold,
        text_threshold=args.text_threshold,
        device=args.device,
    )
    if len(boxes) == 0:
        raise SystemExit("没有检测框：请检查Prompt、阈值或输入图像")

    height, width = image_rgb.shape[:2]
    boxes_xyxy = boxes.clone()
    boxes_xyxy[:, 0] = (boxes[:, 0] - boxes[:, 2] / 2.0) * width
    boxes_xyxy[:, 1] = (boxes[:, 1] - boxes[:, 3] / 2.0) * height
    boxes_xyxy[:, 2] = (boxes[:, 0] + boxes[:, 2] / 2.0) * width
    boxes_xyxy[:, 3] = (boxes[:, 1] + boxes[:, 3] / 2.0) * height

    sam = sam_model_registry[args.sam_type](checkpoint=args.sam_checkpoint)
    sam.to(device=args.device)
    predictor = SamPredictor(sam)
    predictor.set_image(image_rgb)
    transformed_boxes = predictor.transform.apply_boxes_torch(
        boxes_xyxy.to(args.device), image_rgb.shape[:2]
    )
    masks, _, _ = predictor.predict_torch(
        point_coords=None,
        point_labels=None,
        boxes=transformed_boxes,
        multimask_output=False,
    )
    combined_mask = masks[:, 0].any(dim=0).cpu().numpy().astype(np.uint8) * 255
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(output_path), combined_mask)
    for phrase, score in zip(phrases, scores):
        print(f"检测：{phrase}，置信度={float(score):.3f}")
    print(f"Mask已保存：{output_path}")


if __name__ == "__main__":
    main()
