from ultralytics import YOLO
import cv2
import numpy as np

model = YOLO('yolov8n-seg.pt')
bottle_class_index = 39
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    results = model(frame)
    masks = results[0].masks
    bottle_masks = []

    if masks is not None:
        for i, cls in enumerate(results[0].boxes.cls):
            if int(cls) == bottle_class_index:
                mask = masks.data[i].cpu().numpy().astype('uint8')
                mask_resized = cv2.resize(mask, (frame.shape[1], frame.shape[0]))
                bottle_masks.append(mask_resized)

    overlay = frame.copy()
    if bottle_masks:
        for mask in bottle_masks:
            moments = cv2.moments(mask, binaryImage=True)
            if moments["m00"] != 0:
                cx = int(moments["m10"] / moments["m00"])
                cy = int(moments["m01"] / moments["m00"])
            else:
                cx, cy = 0, 0

            if moments["mu20"] - moments["mu02"] != 0:
                angle = 0.5 * np.arctan2(2 * moments["mu11"], moments["mu20"] - moments["mu02"])
                angle_deg = np.degrees(angle)
            else:
                angle_deg = 0

            cv2.circle(overlay, (cx, cy), 5, (0, 0, 255), -1)
            length = 50
            end_x = int(cx + length * np.cos(angle))
            end_y = int(cy + length * np.sin(angle))
            cv2.arrowedLine(overlay, (cx, cy), (end_x, end_y), (255, 0, 0), 2, tipLength=0.2)
            cv2.putText(overlay, f"Centroid: ({cx}, {cy})", (cx + 10, cy - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(overlay, f"Angle: {angle_deg:.2f} deg", (cx + 10, cy + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    cv2.imshow("Bottle Detection", overlay)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

