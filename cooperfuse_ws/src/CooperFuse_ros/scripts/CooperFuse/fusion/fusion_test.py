import numpy as np
from scipy.optimize import linear_sum_assignment

def bbx_iou_0(bbx1,bbx2):
    x1, y1, _, l1, w1, _, _, _ = bbx1
    x2, y2, _, l2, w2, _, _, _ = bbx2

    x_overlap = max(0, min(x1 + l1 / 2, x2 + l2 / 2) - max(x1 - l1 / 2, x2 - l2 / 2))
    y_overlap = max(0, min(y1 + w1 / 2, y2 + w2 / 2) - max(y1 - w1 / 2, y2 - w2 / 2))

    intersection = x_overlap * y_overlap
    area1 = l1 * w1
    area2 = l2 * w2
    union = area1 + area2 - intersection

    iou = intersection / union
    return iou 

def bbx_iou(bbx1,bbx2):
    m1_x, m1_y, _, m1_l, m1_w, _, _, _ = bbx1
    m2_x, m2_y, _, m2_l, m2_w, _, _, _ = bbx2
    
    x1 = m1_x - m1_l / 2.0
    y1 = m1_y - m1_w / 2.0
    x2 = m1_x + m1_l / 2.0
    y2 = m1_y + m1_w / 2.0
    x3 = m2_x - m2_l / 2.0
    y3 = m2_y - m2_w / 2.0
    x4 = m2_x + m2_l / 2.0
    y4 = m2_y + m2_w / 2.0

    # calculate intersection area
    x_left = max(x1, x3)
    y_top = max(y1, y3)
    x_right = min(x2, x4)
    y_bottom = min(y2, y4)
    if x_right < x_left or y_bottom < y_top:
        return 0.0
    intersection_area = (x_right - x_left) * (y_bottom - y_top)

    # calculate union area
    union_area = (x2 - x1) * (y2 - y1) + (x4 - x3) * (y4 - y3) - intersection_area

    # calculate intersection over union
    if union_area == 0.0:
        iou = 0.0
    else:
        iou = intersection_area / union_area

    return iou

def sum_unknown_number_of_values(*args):
    total_sum = sum(args)
    return total_sum

def fusioned_bbx(bbx1,bbx2):
    m1_x, m1_y, m1_yaw, m1_l, m1_w, m1_ds, m1_kds, m1_ss = bbx1
    m2_x, m2_y, m2_yaw, m2_l, m2_w, m2_ds, m2_kds, m2_ss = bbx2
    x_fuse, y_fuse = [0, 0], [0, 0]
    yaw_fuse, l_fuse, w_fuse = 0, 0, 0
    
    ds_sum = sum_unknown_number_of_values(m1_ds,m2_ds)
    kds_sum = sum_unknown_number_of_values(m1_kds, m2_kds)
    ss_sum = sum_unknown_number_of_values(m1_ss, m2_ss)  
    
    x_fuse = (m1_ds/ds_sum) * m1_x + (m2_ds/ds_sum) * m2_x
    y_fuse = (m1_ds/ds_sum) * m1_y + (m2_ds/ds_sum) * m2_y
    
    yaw_fuse = (m1_kds/ds_sum) * m1_yaw + (m2_kds/ds_sum) * m2_yaw
    
    l_fuse = (m1_ss/ss_sum) * m1_l + (m2_ss/ss_sum) * m2_l
    w_fuse = (m1_ss/ss_sum) * m1_w + (m2_ss/ss_sum) * m2_w
    
    fused_bbx = [x_fuse, y_fuse, yaw_fuse, l_fuse, w_fuse, 0, 0, 0]
    return fused_bbx 

def calculate_iou_matrix(boxes_list):
    num_boxes = len(boxes_list)
    iou_matrix = np.zeros((num_boxes, num_boxes))
    # iou_matrix = [[0.0] * num_boxes for _ in range(num_boxes)]
    
    for i in range(num_boxes):
        for j in range(i, num_boxes):
            if i == j:
                iou = 1
                # iou_matrix[i][j] = iou
                iou_matrix[i, j] = iou
            else:
                iou = bbx_iou(boxes_list[i], boxes_list[j])
                # iou_matrix[i][j] = iou
                # iou_matrix[j][i] = iou
                iou_matrix[i, j] = iou
                iou_matrix[j, i] = iou
    
    return iou_matrix     

def find_together_boxes_0(iou_matrix):
    together_indices = np.where((iou_matrix > 0) & (iou_matrix < 1))
    together_pairs = []

    for i, j in zip(together_indices[0], together_indices[1]):
        pair = (i, j)
        if pair not in together_pairs and (j, i) not in together_pairs:
            together_pairs.append(pair)

    return together_pairs

def find_together_groups_0(iou_matrix):
    num_boxes = iou_matrix.shape[0]
    visited = [False] * num_boxes
    together_groups = []

    def dfs(node, group):
        visited[node] = True
        group.append(node)

        for i in range(num_boxes):
            if not visited[i] and iou_matrix[node, i] > 0:
                dfs(i, group)

    for i in range(num_boxes):
        if not visited[i]:
            group = []
            dfs(i, group)
            together_groups.append(group)

    return together_groups

# DFS to find connected boxes
def dfs(node, visited, matrix):
    # Threshold for clustering boxes
    threshold = 0.2
    stack = [node]
    cluster = []
    while stack:
        current = stack.pop()
        if not visited[current]:
            visited[current] = True
            cluster.append(current)
            neighbors = np.where(matrix[current] > threshold)[0]
            for neighbor in neighbors:
                if not visited[neighbor]:
                    stack.append(neighbor)
    return cluster

def find_clusters(iou_matrix):
    num_boxes = iou_matrix.shape[0]
    visited = [False] * num_boxes
    clusters = []
    for i in range(num_boxes):
        if not visited[i]:
            clusters.append(dfs(i, visited, iou_matrix))
    return clusters

# Function to perform fusion operation
def fusion(bboxes):
    # Implement your fusion logic here
    # This function should take a list of bounding boxes and perform fusion
    
    # Placeholder example: concatenate the bounding box names
    fused_bbx = "_".join(bboxes)
    return fused_bbx

def sum_unknown_number_of_values(*args):
    return sum(args)

def fusioned_bbx_multi(bbxs):
    num_bbx = len(bbxs)
    if num_bbx == 0:
        return None  # No bounding boxes to fuse

    # Unpack each bounding box's attributes and calculate sums
    attr_sums = [sum_unknown_number_of_values(*attrs) for attrs in zip(*bbxs)]

    # Calculate fused attributes
    fused_attrs = [sums / num_bbx for sums in attr_sums]

    return fused_attrs

def fusion_multi_bbx(bbxs):
    # bbxs format:
    ## bbxs = [bbx1,bbx2,bbx3,...]
    # bbx format: 
    ##  m1_x, m1_y, m1_yaw, m1_l, m1_w, m1_ds, m1_kds, m1_ss = bbx1
    # Initialize fused values
    x_fuse, y_fuse, yaw_fuse, l_fuse, w_fuse = 0, 0, 0, 0, 0
    
    # Calculate sums for normalization
    ds_sum = sum([bbx[5] for bbx in bbxs])
    kds_sum = sum([bbx[6] for bbx in bbxs])
    ss_sum = sum([bbx[7] for bbx in bbxs])

    # Update fused values based on each bounding box's values
    for bbx in bbxs:
        m_x, m_y, m_yaw, m_l, m_w, m_ds, m_kds, m_ss = bbx
        
        x_fuse += (m_ds/ds_sum) * m_x
        y_fuse += (m_ds/ds_sum) * m_y
        yaw_fuse += (m_kds/kds_sum) * m_yaw
        l_fuse += (m_ss/ss_sum) * m_l
        w_fuse += (m_ss/ss_sum) * m_w

    return [x_fuse, y_fuse, yaw_fuse, l_fuse, w_fuse, 0, 0, 0]

if __name__ == "__main__":
    
    # Example values for bbxs_a1
    bbx_a1_1 = [1.5, 2.0, 0.1, 4.0, 2.0, 0.81, 0.5, 0.62]
    bbx_a1_2 = [3.0, -1.0, -0.2, 3.0, 1.5, 0.71, 0.4, 0.52]
    bbx_a1_3 = [-2.0, 5.0, 0.5, 6.0, 3.0, 0.91, 0.6, 0.72]
    bbxs_a1 = [bbx_a1_1, bbx_a1_2, bbx_a1_3]
    
    # Example values for bbxs_a2
    bbx_a2_1 = [4.0, -2.0, -0.3, 2.5, 1.2, 0.62, 0.3, 0.41]
    bbx_a2_2 = [-1.0, 1.0, 0.0, 2.0, 1.0, 0.72, 0.5, 0.61]
    bbx_a2_3 = [0.0, 3.0, 0.1, 3.5, 1.8, 0.82, 0.7, 0.81]
    bbx_a2_4 = [-5.0, 7.0, 0.8, 7.0, 3.5, 0.22, 0.1, 0.31]
    bbxs_a2 = [bbx_a2_1, bbx_a2_2, bbx_a2_3, bbx_a2_4]
    
    bbxs_comb = bbxs_a1 + bbxs_a2
    
    iou_matrix = calculate_iou_matrix(bbxs_comb)
    # matched_indices = linear_sum_assignment(-iou_matrix)
    
    # test1
    iou_matrix = np.array([
        [1.0, 0.0, 0.6, 0.1, 0.0],
        [0.0, 1.0, 0.0, 0.8, 0.0],
        [0.6, 0.0, 1.0, 0.0, 0.0],
        [0.1, 0.8, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 1.0]
    ])
    
    # test2
    iou_matrix = np.array([
    [1.0, 0.0, 0.6, 0.0, 0.7],
    [0.0, 1.0, 0.0, 0.8, 0.0],
    [0.6, 0.0, 1.0, 0.0, 0.88],
    [0.0, 0.8, 0.0, 1.0, 0.0],
    [0.7, 0.0, 0.88, 0.0, 1.0]
    ])
    
    together_pairs = find_clusters(iou_matrix)
    print("Pairs of boxes that are together:")
    for pair in together_pairs:
        print(pair)    
    
    
    for pair in together_pairs:
        if len(pair) > 1:
            fused_bbxs = [bbxs_comb[i] for i in pair]
            print("Fused result:", fused_bbxs)
            attris = fusioned_bbx_multi(fused_bbxs)
        else:
            fused_bbxs = [bbxs_comb[i] for i in pair]
            print("Fused result:", fused_bbxs)
            
    
    """
    for i in range(len(bbxs_a1)):
        for j in range()
    """
    
    # bbx_a1_1 - > bbx_a2_2
    # bbx_a1_2 - > bbx_a2_1
    """
    iteration1: 
    rank the length of each bx, and place the bounding box in descending sort
    bx1 = [bx11,bx12,bx13,bx14,bx15]
    bx2 = [bx21,bx22,bx23,bx24]
    bx3 = [bx31,bx32,bx33]
    fuse the bounding box that has IOU>0.5, such as IOU(bx11,bx23)>0.5 and IOU(bx11,bx23)>0.5
    bx11 - bx23 - bx32 - > bf1

    remove the fusion bx 
    bx1 = [bx12,bx13,bx14,bx15]
    bx2 = [bx21,bx22,bx24]
    bx3 = [bx31,bx33]    
    
    iteration2: 
    bx12 -> bf2

    bx1 = [bx13,bx14,bx15]
    bx2 = [bx21,bx22,bx24]
    bx3 = [bx31,bx33]   
    
    bx13 -> bf3
    
    iteration3: 
    bx1 = [bx14,bx15]
    bx2 = [bx21,bx22,bx24]
    bx3 = [bx31,bx33]     

    bx2 = [bx21,bx22,bx24]
    bx1 = [bx14,bx15]
    bx3 = [bx31,bx33]    
    
    bx21 - bx15 -> bf4

    iteration4:
    bx2 = [bx22,bx24]
    bx1 = [bx14]
    bx3 = [bx31,bx33]     
    
    bx22 - bx14 -bx31 -> bf5
    
    bx2 = [bx24]
    bx1 = []
    bx3 = [bx33]  
    
    bx24 - bx33 -> bf6
    """
    
    
    
    
    