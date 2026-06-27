"""
Code description:
Date: 
Author:
"""

if __name__ == '__main__':
    # part 1: generate code
    # original_list = [1,2,3,4,5,6,7,8,9,13,14,15,20,21,24,26,31,33,36,38,43,48,50,55,57,60,61,67,71,74,78,84,85,88,91,95,102]
    original_list = [1, 2, 3, 4, 5, 6, 7, 8, 9, 13, 14, 15, 20, 21, 24, 26, 27, 29, 31, 33, 34, 36, 38, 41, 43, 44, 45, 48, 50, 51, 55, 57, 60, 61, 62, 67, 69, 71, 72, 74, 78, 79, 81, 84, 85, 88, 91, 95, 97, 102]
    modified_list = [x - 1 for x in original_list]

    formatted_output = " || ".join([f"i == {i}" for i in modified_list])
    print(formatted_output)

    # part 2: double-check
    # Given string containing multiple 'i == <number>' conditions
    given_string = "i == 0 || i == 1 || i == 2 || i == 3 || i == 4 || i == 5 || i == 6 || i == 7 || i == 8 || i == 12 || i == 13 || i == 14 || i == 19 || i == 20 || i == 23 || i == 25 || i == 26 || i == 28 || i == 30 || i == 32 || i == 33 || i == 35 || i == 37 || i == 40 || i == 42 || i == 43 || i == 44 || i == 47 || i == 49 || i == 50 || i == 54 || i == 56 || i == 59 || i == 60 || i == 61 || i == 66 || i == 68 || i == 70 || i == 71 || i == 73 || i == 77 || i == 78 || i == 80 || i == 83 || i == 84 || i == 87 || i == 90 || i == 94 || i == 96 || i == 101"

    # Count occurrences of 'i'
    count_i = given_string.count("i == ")
    print(count_i)