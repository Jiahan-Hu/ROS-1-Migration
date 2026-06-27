"""
Code description:
Date: 
Author:
"""

if __name__ == '__main__':
    list1 = [1,2,3,4,5,6,7,8,9,13,14,15,20,21,24,26,31,33,36,38,43,48,50,55,57,60,61,67,71,74,78,84,85,88,91,95,102]
    list2 = [1,4,5,6,14,21,27,29,34,41,44,45,51,55,57,62,67,69,72,74,79,81,84,85,91,95,97]

    # Combine the two lists and remove duplicates using a set
    comb_list = list(set(list1 + list2))

    # Print the merged list
    print(comb_list)
