#!/usr/bin/env python3
import os
from PIL import Image

def generate_pdf():
    # 1. Gather all the relevant tag images in the specified order (ID 0 to 3, then ID 10)
    tag_files = ['tag_00.png', 'tag_01.png', 'tag_02.png', 'tag_03.png', 'tag_10_robot.png']
    
    existing_files = []
    for f in tag_files:
        if os.path.exists(f):
            existing_files.append(f)
        else:
            print(f"Warning: File {f} not found in the current directory.")
            
    if not existing_files:
        print("Error: No tag files found to compile.")
        return

    # A4 dimensions at 300 DPI:
    # Width: 210mm = 8.27 in -> 2480 pixels
    # Height: 297mm = 11.69 in -> 3508 pixels
    DPI = 300
    A4_WIDTH = 2480
    A4_HEIGHT = 3508
    
    # Leave a safe margin (e.g., 20mm) around the page for printer compatibility
    margin_mm = 20
    margin_px = int((margin_mm / 25.4) * DPI) # ~236 pixels
    
    max_w = A4_WIDTH - 2 * margin_px
    max_h = A4_HEIGHT - 2 * margin_px

    pages = []
    print("Preparing tag compilation...")
    for f in existing_files:
        img = Image.open(f)
        
        # Calculate aspect ratio scaling to fit within the margins
        w_ratio = max_w / img.width
        h_ratio = max_h / img.height
        scale = min(w_ratio, h_ratio)
        
        new_size = (int(img.width * scale), int(img.height * scale))
        resized_img = img.resize(new_size, Image.Resampling.LANCZOS)
        
        # Create a blank white A4 page
        page = Image.new("RGB", (A4_WIDTH, A4_HEIGHT), "white")
        
        # Center the image on the page
        x = (A4_WIDTH - new_size[0]) // 2
        y = (A4_HEIGHT - new_size[1]) // 2
        page.paste(resized_img, (x, y))
        
        pages.append(page)
        
    # Save as a multi-page PDF
    output_filename = "tags_id0-3_id10.pdf"
    if pages:
        pages[0].save(
            output_filename,
            "PDF",
            resolution=DPI,
            save_all=True,
            append_images=pages[1:]
        )
        print(f"\n[Success] Generated PDF: {output_filename}\n")
        print("Physical size of the black tag square when printed at 100% scale (no scaling) on A4:")
        
        for f in existing_files:
            img = Image.open(f)
            w_ratio = max_w / img.width
            h_ratio = max_h / img.height
            scale = min(w_ratio, h_ratio)
            
            # The width of the actual black square in pixels (excluding margins and labels):
            # For tag_00 to tag_03: bounding box is (221, 221, 1993, 2340). Tag is 1772 x 1772 px.
            # For tag_10_robot: bounding box is (177, 177, 1594, 1872). Tag is 1417 x 1417 px.
            if "robot" in f:
                tag_sq_orig_px = 1417
            else:
                tag_sq_orig_px = 1772
                
            tag_sq_scaled_px = tag_sq_orig_px * scale
            tag_sq_physical_in = tag_sq_scaled_px / DPI
            tag_sq_physical_cm = tag_sq_physical_in * 2.54
            
            print(f"  - {f:<16}: Square length = {tag_sq_physical_cm:.2f} cm")

if __name__ == "__main__":
    generate_pdf()
