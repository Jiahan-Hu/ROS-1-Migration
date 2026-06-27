import numpy as np

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

def bbx_iou(bbx1,bbx2):
    m1_vid,m1_x, m1_y, _, m1_l, m1_w, _, _, _ = bbx1
    m2_vid,m2_x, m2_y, _, m2_l, m2_w, _, _, _ = bbx2
    
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

# DFS to find connected boxes
def dfs(node, visited, matrix,iou_threshold):
    # Threshold for clustering boxes
    # hyper parameter: iou threshold 
    # threshold = 0.05
    
    stack = [node]
    cluster = []
    while stack:
        current = stack.pop()
        if not visited[current]:
            visited[current] = True
            cluster.append(current)
            neighbors = np.where(matrix[current] > iou_threshold)[0]
            for neighbor in neighbors:
                if not visited[neighbor]:
                    stack.append(neighbor)
    return cluster

def find_clusters(iou_matrix,iou_threshold):
    num_boxes = iou_matrix.shape[0]
    visited = [False] * num_boxes
    clusters = []
    for i in range(num_boxes):
        if not visited[i]:
            clusters.append(dfs(i, visited, iou_matrix,iou_threshold))
    return clusters


    
def fusion_multi_bbx(bbxs):
    # bbxs format:
    ## bbxs = [bbx1,bbx2,bbx3,...]
    # bbx format: 
    ##  m1_x, m1_y, m1_yaw, m1_l, m1_w, m1_ds, m1_kds, m1_ss = bbx1
    # Initialize fused values
    vid,x_fuse, y_fuse, yaw_fuse, l_fuse, w_fuse = 100,0, 0, 0, 0, 0
    
    # Calculate sums for normalization
    ds_sum = sum([bbx[6] for bbx in bbxs])
    # need to modify
    # ss_sum = sum([bbx[7] for bbx in bbxs])
    ls_sum = sum([bbx[7][0] for bbx in bbxs])
    ws_sum = sum([bbx[7][1] for bbx in bbxs])
    
    bbxs = kds_cal(bbxs)

    # Update fused values based on each bounding box's values
    for bbx in bbxs:
        m_vid,m_x, m_y, m_yaw, m_l, m_w, m_ds, m_ss ,m_kds = bbx
        
        assert m_ds != 0.0, "Error：ds cannot be 0. It needs to be none 0."
        assert m_ss != 0.0, "Error：ss cannot be 0. It needs to be none 0."
        assert m_kds != 0.0, "Error：kds cannot be 0. It needs to be none 0."
        
        # do  I need to use kds to constaints x,y as well?  
        #x_fuse += (m_ds/ds_sum) * m_x 
        # y_fuse += (m_ds/ds_sum) * m_y
        
        #x_fuse += m_kds * m_x 
        #y_fuse += m_kds * m_y  
        
        x_fuse += ((m_ds+m_kds)/(ds_sum+1)) * m_x #This is problematic
        y_fuse += ((m_ds+m_kds)/(ds_sum+1)) * m_y #This is problematic 
        
        yaw_fuse += m_kds * m_yaw 
        l_fuse += (m_ss[0]/ls_sum) * m_l
        w_fuse += (m_ss[1]/ws_sum) * m_w

    return [vid,x_fuse, y_fuse, yaw_fuse, l_fuse, w_fuse, 0, 0, 0]

def fuse_multi_bbxs(bbxs_comb,together_pairs):
    fusion_bbxs_final = []
    for pair in together_pairs:
        if len(pair) > 1:
            fused_bbxs = [bbxs_comb[i] for i in pair]
            # print("Fused result:", fused_bbxs)
            fusion_bbx = fusion_multi_bbx(fused_bbxs)
            fusion_bbxs_final.append(fusion_bbx)
        else:
            fused_bbxs = [bbxs_comb[i] for i in pair]
            # print("Fused result:", fused_bbxs)
            fusion_bbx = fused_bbxs[0]
            fusion_bbx[-1] = 0
            fusion_bbx[-2] = 0
            fusion_bbx[-3] = 0
            fusion_bbxs_final.append(fusion_bbx)
    return fusion_bbxs_final

# Compute the softmax values
def softmax(arr):
    exp_values = np.exp(arr)
    return exp_values / np.sum(exp_values)
    
def kds_cal(bbxs):
    
    k = len(bbxs)
    Esum = 0
    
    E_kds = []
    for i,bbx in enumerate(bbxs):
        m_vid,m_x, m_y, m_yaw, m_l, m_w, m_ds, m_ss ,m_kds = bbx
        if type(m_kds) == str:
            if m_kds[0] == 'E':
                m_kds = float(m_kds[1:])
                E_kds.append(m_kds)

    lambd = 0.05  # adjust lambda to adjust the probability distribution
    p_kds = calculate_probabilities(E_kds,lambd)
    
    # print(p_kds)
    
    multi_kds = []
    p_i  = 0
    for bbx in bbxs:
        m_vid,m_x, m_y, m_yaw, m_l, m_w, m_ds, m_ss ,m_kds = bbx
        if m_kds == -1:
            m_kds = 1.0/k
            multi_kds.append(m_kds) 
        else:
            m_kds = p_kds[p_i]
            multi_kds.append(m_kds)
            p_i += 1

        
    # if there are only 2 agent, and one of which is -1, then kds = [0.5,1]
    
    # final kds list
    f_kds_list = softmax(multi_kds)
    for i,f_kds in enumerate(f_kds_list):
        bbxs[i][-1] = f_kds
        
    return bbxs    

def calculate_probabilities(values, lambd):
    probabilities = []
    denominator = np.sum(np.exp(-lambd * np.array(values)))
    
    for x in values:
        numerator = np.exp(-lambd * x)
        probability = numerator / denominator
        probabilities.append(probability)
    
    return probabilities


def test1():
    # Example values for bbxs_a1
    bbx_a1_1 = [311,1.5, 2.0, 0.1, 4.0, 2.0, 0.81, 0.5, 0.62]
    bbx_a1_2 = [312,3.0, -1.0, -0.2, 3.0, 1.5, 0.71, 0.4, 0.52]
    bbx_a1_3 = [313,-2.0, 5.0, 0.5, 6.0, 3.0, 0.91, 0.6, 0.72]
    bbxs_a1 = [bbx_a1_1, bbx_a1_2, bbx_a1_3]
    
    # Example values for bbxs_a2
    bbx_a2_1 = [524,4.0, -2.0, -0.3, 2.5, 1.2, 0.62, 0.3, 0.41]
    bbx_a2_2 = [525,-1.0, 1.0, 0.0, 2.0, 1.0, 0.72, 0.5, 0.61]
    bbx_a2_3 = [526,0.0, 3.0, 0.1, 3.5, 1.8, 0.82, 0.7, 0.81]
    bbx_a2_4 = [527,-5.0, 7.0, 0.8, 7.0, 3.5, 0.22, 0.1, 0.31]
    bbxs_a2 = [bbx_a2_1, bbx_a2_2, bbx_a2_3, bbx_a2_4]
    
    bbxs_comb = bbxs_a1 + bbxs_a2
    
    iou_matrix = calculate_iou_matrix(bbxs_comb)
    
    together_pairs = find_clusters(iou_matrix)
    
    fusion_bbxs_final = fuse_multi_bbxs(bbxs_comb,together_pairs)    

def test2():
    # Example values for bbxs_a1
    bbx_a1_1 = [311,1.5, 2.0, 0.1, 4.0, 2.0, 0.81, [0.78,0.5], 0.62]
    bbx_a1_2 = [312,3.0, -1.0, -0.2, 3.0, 1.5, 0.71, [0.4,0.9], 0.52]
    bbx_a1_3 = [313,-2.0, 5.0, 0.5, 6.0, 3.0, 0.91, [0.6,0.8], 0.72]
    bbxs_a1 = [bbx_a1_1, bbx_a1_2, bbx_a1_3]
    
    # Example values for bbxs_a2
    bbx_a2_1 = [524,4.0, -2.0, -0.3, 2.5, 1.2, 0.62, [0.3,0.6], 0.41]
    bbx_a2_2 = [525,-1.0, 1.0, 0.0, 2.0, 1.0, 0.72, [0.88,0.5], 0.61]
    bbx_a2_3 = [526,0.0, 3.0, 0.1, 3.5, 1.8, 0.82, [0.7,0.89], 0.81]
    bbx_a2_4 = [527,-5.0, 7.0, 0.8, 7.0, 3.5, 0.22, [0.61,0.77], 0.31]
    bbxs_a2 = [bbx_a2_1, bbx_a2_2, bbx_a2_3, bbx_a2_4]
    
    bbxs_comb = bbxs_a1 + bbxs_a2
    
    iou_matrix = calculate_iou_matrix(bbxs_comb)
    
    together_pairs = find_clusters(iou_matrix,iou_threshold=0.05) 
    
    fusion_bbxs_final = fuse_multi_bbxs(bbxs_comb,together_pairs) 

def test3():
    # test3
    # Example values for bbxs_a1
    bbx_a1_1 = [311,1.5, 2.0, 0.1, 4.0, 2.0, 0.81, [0.78,0.5], 'E62']
    bbx_a1_2 = [312,3.0, -1.0, -0.2, 3.0, 1.5, 0.71, [0.4,0.9], 'E41']
    bbx_a1_3 = [313,-2.0, 5.0, 0.5, 6.0, 3.0, 0.91, [0.6,0.8], 'E72']
    bbxs_a1 = [bbx_a1_1, bbx_a1_2, bbx_a1_3]
    
    # Example values for bbxs_a2
    bbx_a2_1 = [524,4.0, -2.0, -0.3, 2.5, 1.2, 0.62, [0.3,0.6], 'E411']
    bbx_a2_2 = [525,-1.0, 1.0, 0.0, 2.0, 1.0, 0.72, [0.88,0.5], -1]
    bbx_a2_3 = [526,0.0, 3.0, 0.1, 3.5, 1.8, 0.82, [0.7,0.89], 'E8.1']
    bbx_a2_4 = [527,-5.0, 7.0, 0.8, 7.0, 3.5, 0.22, [0.61,0.77], 'E0.31']
    bbxs_a2 = [bbx_a2_1, bbx_a2_2, bbx_a2_3, bbx_a2_4]
    
    bbxs_comb = bbxs_a1 + bbxs_a2
    
    iou_matrix = calculate_iou_matrix(bbxs_comb)
    
    together_pairs = find_clusters(iou_matrix,iou_threshold=0.05) 
    
    fusion_bbxs_final = fuse_multi_bbxs(bbxs_comb,together_pairs) 

def test4():
    multi = [0.00000001,0.99999]
    multi = [0.48175810020426185, 0.5182418997957381]
    f_p = softmax(multi)
    print(f_p)
if __name__ == "__main__":


    E_kds = [0.0,0.0]
    lambd = 0.05 

    p_kds = calculate_probabilities(E_kds,lambd)
    print("p_kds:",p_kds)






    


