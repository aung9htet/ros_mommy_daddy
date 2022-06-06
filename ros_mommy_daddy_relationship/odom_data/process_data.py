import numpy as np

def process_data():
    file = input("Please type in the file name to process: ") # request input from user of file name
    get_data = np.load(file) # will return an np array of tuples that is of (pos_x, pos_y)
    print(get_data)

if __name__ == "__main__":
    process_data()