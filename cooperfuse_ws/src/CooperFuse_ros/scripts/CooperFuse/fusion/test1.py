def calculate_iou(box1, box2):
    # Calculate coordinates of intersection
    x1 = max(box1[0], box2[0])
    y1 = max(box1[1], box2[1])
    x2 = min(box1[2], box2[2])
    y2 = min(box1[3], box2[3])
    
    # Calculate intersection area
    intersection_area = max(0, x2 - x1) * max(0, y2 - y1)
    
    # Calculate union area
    box1_area = (box1[2] - box1[0]) * (box1[3] - box1[1])
    box2_area = (box2[2] - box2[0]) * (box2[3] - box2[1])
    union_area = box1_area + box2_area - intersection_area
    
    # Calculate IOU
    iou = intersection_area / union_area
    return iou

def find_matching_boxes(boxes_list):
    matching_boxes = []
    num_groups = len(boxes_list)
    
    for group_idx in range(num_groups):
        group_boxes = boxes_list[group_idx]
        for box_idx, box in enumerate(group_boxes):
            for other_group_idx in range(group_idx + 1, num_groups):
                for other_box in boxes_list[other_group_idx]:
                    iou = calculate_iou(box, other_box)
                    if iou > 0.5:
                        matching_boxes.append((group_idx, box_idx, other_group_idx, other_box))
    
    return matching_boxes

# Example bounding boxes for two groups
group1_boxes = [
    [10, 10, 50, 50],
    [30, 30, 70, 70]
]

group2_boxes = [
    [10, 10, 60, 60],
    [80, 80, 120, 120]
]

boxes_list = [group1_boxes, group2_boxes]
matching_boxes = find_matching_boxes(boxes_list)
print(matching_boxes)