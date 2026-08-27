#!/usr/bin/env python3
"""
SAM 2.1 Segmentation Test for 4GB VRAM (RTX 3050 Laptop)
Tests text-prompted segmentation for path detection.
"""
import os
import sys
import cv2
import numpy as np
import torch
from PIL import Image
import matplotlib.pyplot as plt

# Memory optimization for 4GB VRAM
os.environ['PYTORCH_CUDA_ALLOC_CONF'] = 'expandable_segments:True'
torch.backends.cudnn.benchmark = True

def check_gpu():
    """Check GPU availability and memory"""
    if torch.cuda.is_available():
        gpu_name = torch.cuda.get_device_name(0)
        total_mem = torch.cuda.get_device_properties(0).total_memory / 1024**3
        print(f"GPU: {gpu_name}")
        print(f"Total VRAM: {total_mem:.1f} GB")
        
        # Clear cache
        torch.cuda.empty_cache()
        allocated = torch.cuda.memory_allocated(0) / 1024**3
        reserved = torch.cuda.memory_reserved(0) / 1024**3
        print(f"Allocated: {allocated:.2f} GB, Reserved: {reserved:.2f} GB")
        return True
    else:
        print("CUDA not available, using CPU")
        return False

def load_sam_model(model_size="tiny"):
    """
    Load SAM 2.1 model optimized for low VRAM.
    model_size: "tiny", "small", "base_plus", "large"
    """
    try:
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
    except ImportError:
        print("SAM 2 not installed. Installing...")
        import subprocess
        subprocess.run([sys.executable, "-m", "pip", "install", "git+https://github.com/facebookresearch/segment-anything-2.git"])
        from sam2.build_sam import build_sam2
        from sam2.sam2_image_predictor import SAM2ImagePredictor
    
    # Model config mapping
    configs = {
        "tiny": "configs/sam2.1/sam2.1_hiera_t.yaml",
        "small": "configs/sam2.1/sam2.1_hiera_s.yaml",
        "base_plus": "configs/sam2.1/sam2.1_hiera_b+.yaml",
        "large": "configs/sam2.1/sam2.1_hiera_l.yaml",
    }
    
    checkpoints = {
        "tiny": "sam2.1_hiera_tiny.pt",
        "small": "sam2.1_hiera_small.pt", 
        "base_plus": "sam2.1_hiera_base_plus.pt",
        "large": "sam2.1_hiera_large.pt",
    }
    
    config_path = configs[model_size]
    checkpoint_name = checkpoints[model_size]
    
    print(f"Loading SAM 2.1 {model_size.upper()}...")
    print(f"Config: {config_path}")
    print(f"Checkpoint: {checkpoint_name}")
    
    # Build model
    sam2_model = build_sam2(config_path, checkpoint_name, device="cuda")
    
    # Memory optimizations
    sam2_model.eval()
    
    # Use half precision for memory savings
    if torch.cuda.is_available():
        sam2_model = sam2_model.half()
    
    predictor = SAM2ImagePredictor(sam2_model)
    print(f"Model loaded successfully!")
    
    return predictor

def load_mobile_sam():
    """Alternative: Load MobileSAM (lighter)"""
    try:
        from mobile_sam import sam_model_registry, SamPredictor
    except ImportError:
        print("MobileSAM not installed. Installing...")
        import subprocess
        subprocess.run([sys.executable, "-m", "pip", "install", "git+https://github.com/ChaoningZhang/MobileSAM.git"])
        from mobile_sam import sam_model_registry, SamPredictor
    
    model_type = "vit_t"
    checkpoint = "mobile_sam.pt"
    
    print(f"Loading MobileSAM ({model_type})...")
    sam = sam_model_registry[model_type](checkpoint=checkpoint)
    sam.to(device="cuda")
    sam.eval()
    
    if torch.cuda.is_available():
        sam = sam.half()
    
    predictor = SamPredictor(sam)
    print("MobileSAM loaded!")
    return predictor

def run_text_prompt_segmentation(predictor, image, text_prompts, output_dir):
    """
    Run segmentation with text prompts.
    Note: SAM 2 doesn't natively support text prompts.
    We'll use a workaround: CLIP + SAM or use point/box prompts derived from text.
    """
    # SAM 2 doesn't have native text prompt support.
    # Options:
    # 1. Use GroundingDINO + SAM (text -> box -> SAM)
    # 2. Use CLIP to find regions, then SAM with points
    # 3. Use point prompts manually placed on path
    
    # For now, we'll use a hybrid approach:
    # - Use point prompts placed on visible path areas
    # - Or use automatic mask generation (SAM's everything mode)
    
    pass

def run_automatic_mask_generation(predictor, image, output_dir):
    """Generate all masks automatically (SAM's 'everything' mode)"""
    print("Running automatic mask generation...")
    
    # Set image
    predictor.set_image(image)
    
    # Generate masks - SAM 2 automatic mask generation
    # Note: SAM2ImagePredictor doesn't have generate() method
    # We'll use point grid sampling instead
    
    h, w = image.shape[:2]
    
    # Create grid of points
    points_per_side = 16  # 16x16 = 256 points
    point_grids = []
    for i in range(points_per_side):
        for j in range(points_per_side):
            x = (j + 0.5) * w / points_per_side
            y = (i + 0.5) * h / points_per_side
            point_grids.append([x, y])
    
    point_coords = np.array(point_grids)
    point_labels = np.ones(len(point_grids))
    
    # Predict masks for all points
    masks, scores, logits = predictor.predict(
        point_coords=point_coords,
        point_labels=point_labels,
        multimask_output=True,
    )
    
    return masks, scores, logits

def run_point_prompt_segmentation(predictor, image, points, labels, output_dir):
    """Run segmentation with point prompts"""
    print(f"Running point-prompt segmentation with {len(points)} points...")
    
    predictor.set_image(image)
    
    masks, scores, logits = predictor.predict(
        point_coords=np.array(points),
        point_labels=np.array(labels),
        multimask_output=True,
    )
    
    return masks, scores, logits

def run_box_prompt_segmentation(predictor, image, boxes, output_dir):
    """Run segmentation with box prompts"""
    print(f"Running box-prompt segmentation with {len(boxes)} boxes...")
    
    predictor.set_image(image)
    
    # SAM 2 box prompt format: [x1, y1, x2, y2]
    masks, scores, logits = predictor.predict(
        box=np.array(boxes),
        multimask_output=True,
    )
    
    return masks, scores, logits

def visualize_masks(image, masks, scores, output_path, prompt_type="auto"):
    """Create visualization overlay"""
    h, w = image.shape[:2]
    overlay = image.copy()
    
    # Color map for different masks
    colors = plt.cm.tab20(np.linspace(0, 1, 20))[:, :3] * 255
    colors = colors.astype(np.uint8)
    
    # Combine top masks
    combined_mask = np.zeros((h, w), dtype=np.uint8)
    
    for i, (mask, score) in enumerate(zip(masks, scores)):
        if score > 0.5:  # Confidence threshold
            color = colors[i % len(colors)]
            mask_binary = (mask > 0).astype(np.uint8)
            combined_mask = cv2.bitwise_or(combined_mask, mask_binary * 255)
            
            # Overlay with transparency
            color_mask = np.zeros_like(image)
            color_mask[mask_binary > 0] = color
            overlay = cv2.addWeighted(overlay, 0.7, color_mask, 0.3, 0)
    
    # Create side-by-side
    combined = np.hstack([image, overlay])
    
    # Add text info
    cv2.putText(combined, f"Original | {prompt_type} (masks: {len(masks)})", 
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,255), 2)
    
    cv2.imwrite(output_path, combined)
    print(f"Saved visualization: {output_path}")
    
    return combined, combined_mask

def save_individual_masks(masks, scores, output_dir, prefix="mask"):
    """Save individual mask arrays"""
    for i, (mask, score) in enumerate(zip(masks, scores)):
        if score > 0.3:
            mask_path = os.path.join(output_dir, f"{prefix}_{i}_score{score:.3f}.png")
            cv2.imwrite(mask_path, (mask * 255).astype(np.uint8))
            
            npy_path = os.path.join(output_dir, f"{prefix}_{i}_score{score:.3f}.npy")
            np.save(npy_path, mask)

def main():
    # Setup
    image_path = "/home/prem/terralink/tests/seg_test.png"
    output_dir = "/home/prem/terralink/tests/results/sam_test"
    os.makedirs(output_dir, exist_ok=True)
    
    print("=" * 60)
    print("SAM SEGMENTATION TEST - RTX 3050 4GB VRAM")
    print("=" * 60)
    
    # Check GPU
    has_cuda = check_gpu()
    
    # Load image
    image = cv2.imread(image_path)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    h, w = image.shape[:2]
    print(f"Image: {w}x{h}")
    
    # Try SAM 2.1 Tiny first (best for 4GB VRAM)
    try:
        predictor = load_sam_model("tiny")
        model_name = "SAM2.1_Tiny"
    except Exception as e:
        print(f"SAM 2.1 Tiny failed: {e}")
        print("Trying MobileSAM...")
        try:
            predictor = load_mobile_sam()
            model_name = "MobileSAM"
        except Exception as e2:
            print(f"MobileSAM failed: {e2}")
            print("Trying FastSAM...")
            # Could add FastSAM here
            return
    
    print(f"\nUsing: {model_name}")
    
    # Test 1: Automatic mask generation (grid of points)
    print("\n" + "="*50)
    print("TEST 1: Automatic Mask Generation (Grid Points)")
    print("="*50)
    
    try:
        masks, scores, logits = run_automatic_mask_generation(predictor, image_rgb, output_dir)
        print(f"Generated {len(masks)} masks")
        print(f"Score range: {scores.min():.3f} - {scores.max():.3f}")
        
        vis, combined = visualize_masks(image, masks, scores, 
                                         os.path.join(output_dir, f"{model_name}_auto_masks.png"),
                                         "Auto Grid")
        save_individual_masks(masks, scores, output_dir, "auto")
        
    except Exception as e:
        print(f"Auto generation failed: {e}")
    
    # Test 2: Point prompts on path areas (manual)
    print("\n" + "="*50)
    print("TEST 2: Point Prompts on Path Areas")
    print("="*50)
    
    # Manual points on visible path areas in the image
    # These are approximate - in practice you'd click or use detection
    path_points = [
        [w//2, h//2],           # Center
        [w//3, h//2],           # Left of center
        [2*w//3, h//2],         # Right of center
        [w//2, h//3],           # Upper
        [w//2, 2*h//3],         # Lower
    ]
    path_labels = [1] * len(path_points)  # 1 = foreground
    
    try:
        masks, scores, logits = run_point_prompt_segmentation(
            predictor, image_rgb, path_points, path_labels, output_dir)
        print(f"Generated {len(masks)} masks from points")
        
        vis, combined = visualize_masks(image, masks, scores,
                                         os.path.join(output_dir, f"{model_name}_point_masks.png"),
                                         "Point Prompts")
        save_individual_masks(masks, scores, output_dir, "point")
        
    except Exception as e:
        print(f"Point prompts failed: {e}")
    
    # Test 3: Box prompts around path region
    print("\n" + "="*50)
    print("TEST 3: Box Prompts Around Path")
    print("="*50)
    
    # Box around central path area
    path_boxes = [
        [w//4, h//4, 3*w//4, 3*h//4],  # Central region
        [w//3, h//3, 2*w//3, 2*h//3],  # Tighter center
    ]
    
    try:
        masks, scores, logits = run_box_prompt_segmentation(
            predictor, image_rgb, path_boxes, output_dir)
        print(f"Generated {len(masks)} masks from boxes")
        
        vis, combined = visualize_masks(image, masks, scores,
                                         os.path.join(output_dir, f"{model_name}_box_masks.png"),
                                         "Box Prompts")
        save_individual_masks(masks, scores, output_dir, "box")
        
    except Exception as e:
        print(f"Box prompts failed: {e}")
    
    # Summary
    print("\n" + "="*60)
    print("TEST COMPLETE")
    print("="*60)
    print(f"Results saved to: {output_dir}")
    print("Files:")
    for f in os.listdir(output_dir):
        print(f"  {f}")

if __name__ == "__main__":
    main()