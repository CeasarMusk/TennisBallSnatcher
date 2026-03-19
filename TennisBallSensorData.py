
#pip install ultralytics

from ultralytics import YOLO


model = YOLO('yolov8n.pt') 

results = model.predict(source="0", imgsz=640, stream=True, conf=0.5)

for r in results:
    # A. Get Image Dimensions
    img_h, img_w = r.orig_shape 
    img_center = img_w / 2
    
    # B. Loop through detections (the ball)
    for box in r.boxes:
        # Get coordinates in [xmin, ymin, xmax, ymax]
        coords = box.xyxy[0].tolist()
        xmin, ymin, xmax, ymax = coords
        
        # C. Calculate the variables
        pixel_width = xmax - xmin
        ball_center_x = (xmin + xmax) / 2
        
        # --- DATA FOR NAVIGATION  ---
        print(f"Frame Width: {img_w} px")
        print(f"Ball Pixel Width: {pixel_width:.2f}")
        print(f"X-Min: {xmin:.2f} | X-Max: {xmax:.2f}")
        print(f"Horizontal Offset: {ball_center_x - img_center:.2f}")
        print("-" * 20)
        
       
